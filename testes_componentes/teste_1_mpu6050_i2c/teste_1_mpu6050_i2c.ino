/*
  =====================================================================
   TESTE 1 - MPU6050 (GY-521 azul)  |  ESP32  |  IC IFSP-PEP
  =====================================================================
   OBJETIVO
     Conferir se o MPU6050 esta ligado certo e mandando sinal bom,
     em 4 etapas, cada uma com resultado "OK" ou "FALHOU":
       1. Barramento I2C : procura dispositivos em D21/D22
       2. Identidade     : le o registrador WHO_AM_I
       3. Parado (2 s)   : mede gravidade, ruido e bias do giroscopio
       4. Ao vivo        : mostra angulo de inclinacao (roll/pitch) e
                           temperatura; incline a viga e veja mudar.

   LIGACAO (a mesma da bancada) - SO ESTES 4 FIOS
     MPU VCC -> 3V3 (trilho +)      MPU SCL -> D22
     MPU GND -> GND (trilho -)      MPU SDA -> D21
     AD0, XDA, XCL, INT -> soltos   (endereco 0x68)
   NAO precisa de bateria, ESCs nem motores. So o USB.
   Monitor Serial: 115200 baud.
   Nenhum pino do BMI323, do GY-601N1 ou dos ESCs e usado neste teste.

   PINOS USADOS NESTE CODIGO
     D21 = SDA (dado I2C)      D22 = SCL (clock I2C)
     (Serial0/USB para o Monitor Serial)

   STATUS (04/11/2026)
     O MPU6050 em D21/D22 funcionou nos testes de 03/11/2026.

   REFERENCIAS (fontes usadas para os registradores e valores)
     - InvenSense, "MPU-6000/MPU-6050 Register Map and Descriptions",
       RM-MPU-6000A-00 rev. 4.2 (registradores 0x3B, 0x6B, 0x75).
     - InvenSense, "MPU-6000/MPU-6050 Product Specification" rev. 3.4
       (escala padrao: +-2 g = 16384 LSB/g ; +-250 dps = 131 LSB/(deg/s)).
     - Biblioteca ElectronicCats/mpu6050 (base i2cdevlib, J. Rowberg):
       https://github.com/ElectronicCats/mpu6050
     - dRehmFlight (N. Rehm), funcao getIMUdata():
       https://github.com/nickrehm/dRehmFlight
   Escolhemos ler os registradores DIRETO (sem biblioteca) para nao
   depender de versao de biblioteca - igual ao firmware da balanca.
  =====================================================================
*/
#include <Wire.h>   // biblioteca padrao do Arduino/ESP32 para I2C

// Estatistica guarda as somas para calcular media e desvio padrao sem guardar
// todas as amostras: soma[k] = soma dos valores, somaQuadrados[k] = soma dos quadrados,
// quantidade = numero de amostras. Indices k: 0..2 = acel X,Y,Z (g);
// 3..5 = giro X,Y,Z (graus/s, ja sem offset); 6 = modulo da aceleracao (g).
struct Estatistica { double soma[8], somaQuadrados[8]; int quantidade; };   // (no topo: exigencia do Arduino IDE)

// ----------------------- configuracao ---------------------------------
#define PINO_SDA   21        // fio de dado do I2C
#define PINO_SCL   22        // fio de clock do I2C
#define ENDERECO_MPU 0x68    // endereco do MPU6050 (AD0 em GND/solto)

// registradores do MPU6050 (do Register Map da InvenSense)
#define REG_PWR_MGMT_1  0x6B // gerenciamento de energia (acorda o chip)
#define REG_ACCEL_XOUT  0x3B // inicio dos 14 bytes de dados
#define REG_WHO_AM_I    0x75 // identidade do chip

#define NOME_SENSOR "MPU6050_I2C"     // nome que aparece na linha RESUMO_CSV
#define PERIODO_US 5000               // pausa entre leituras na calibracao (5 ms = 200 Hz)
#define AMOSTRAS_AQUECIMENTO 100      // leituras descartadas antes de calibrar
#define AMOSTRAS_CALIBRACAO 400       // 400 amostras x 5 ms = 2 s
#define AMOSTRAS_CONFERENCIA 200      // segunda janela, para conferir a calibracao

// fatores de escala na configuracao padrao de fabrica
const float LSB_POR_G   = 16384.0;  // acelerometro +-2 g
const float LSB_POR_DPS = 131.0;    // giroscopio +-250 graus/s

// Ultima leitura (g, graus/s, graus C)
float aceleracaoX, aceleracaoY, aceleracaoZ;
float giroX, giroY, giroZ;
float temperaturaC;

// Calibracao: o giro em repouso vira offset e a viga nivelada vira o "zero" de roll/pitch.
float offsetGiroX = 0, offsetGiroY = 0, offsetGiroZ = 0;
float offsetRoll = 0, offsetPitch = 0;

// ----------------------- funcoes auxiliares ---------------------------

// Escreve 1 byte "valor" no registrador. Devolve true se o chip respondeu (ACK);
// se der false, problema de fio/endereco.
bool escreverRegistrador(uint8_t registrador, uint8_t valor) {
  Wire.beginTransmission(ENDERECO_MPU);   // inicia transmissao para o MPU
  Wire.write(registrador);                // 1o byte: qual registrador
  Wire.write(valor);                      // 2o byte: o valor a gravar
  return Wire.endTransmission() == 0;     // 0 = sucesso
}

// Le "quantidade" bytes a partir de "registrador" para o vetor "destino".
bool lerRegistradores(uint8_t registrador, uint8_t* destino, uint8_t quantidade) {
  bool leuTudo = false;
  uint8_t i = 0;

  Wire.beginTransmission(ENDERECO_MPU);
  Wire.write(registrador);                                          // aponta para o registrador inicial
  if (Wire.endTransmission(false) == 0) {                           // false = sem STOP (repeated start)
    if (Wire.requestFrom((uint8_t)ENDERECO_MPU, quantidade) == quantidade) {   // pede os bytes
      for (i = 0; i < quantidade; i++) destino[i] = Wire.read();    // copia para o vetor
      leuTudo = true;
    }
  }
  return leuTudo;
}

// Junta 2 bytes (alto, baixo) num numero com sinal de 16 bits.
// O MPU6050 manda o byte ALTO primeiro (big-endian).
int16_t juntarBytes(uint8_t alto, uint8_t baixo) {
  return (int16_t)((alto << 8) | baixo);
}

// Le os 14 bytes (acel 6 + temp 2 + giro 6) e converte.
// Ordem dos bytes a partir de 0x3B: ACC_X(2) ACC_Y(2) ACC_Z(2) TEMP(2) GYR_X(2) GYR_Y(2) GYR_Z(2)
bool lerSensor() {
  uint8_t bytes[14];
  bool leu = lerRegistradores(REG_ACCEL_XOUT, bytes, 14);   // leitura em rajada de 14 bytes

  if (leu) {
    aceleracaoX = juntarBytes(bytes[0], bytes[1]) / LSB_POR_G;     // bruto -> g
    aceleracaoY = juntarBytes(bytes[2], bytes[3]) / LSB_POR_G;
    aceleracaoZ = juntarBytes(bytes[4], bytes[5]) / LSB_POR_G;
    temperaturaC = juntarBytes(bytes[6], bytes[7]) / 340.0 + 36.53; // formula do Register Map: T = valor/340 + 36.53
    giroX = juntarBytes(bytes[8], bytes[9]) / LSB_POR_DPS;         // bruto -> graus/s
    giroY = juntarBytes(bytes[10], bytes[11]) / LSB_POR_DPS;
    giroZ = juntarBytes(bytes[12], bytes[13]) / LSB_POR_DPS;
  }
  return leu;
}

// Imprime "[ OK ] etapa" ou "[FALHOU] etapa" + dica.
void mostrarResultado(const char* etapa, bool passou, const char* dica) {
  Serial.printf("[%s] %s\n", passou ? " OK  " : "FALHOU", etapa);
  if (!passou) Serial.printf("        -> %s\n", dica);
}

// Roll pela direcao da gravidade (graus).
float calcularRollAcel() {
  return atan2(aceleracaoY, aceleracaoZ) * 180.0 / PI;
}

// Pitch pela direcao da gravidade (graus).
float calcularPitchAcel() {
  return atan2(-aceleracaoX, sqrt(aceleracaoY * aceleracaoY + aceleracaoZ * aceleracaoZ)) * 180.0 / PI;
}

void zerarEstatistica(Estatistica& est) {
  memset(&est, 0, sizeof(est));
}

// Acumula a leitura atual (giro ja sem offset).
void somarLeitura(Estatistica& est) {
  float valores[7];
  int k = 0;

  valores[0] = aceleracaoX;
  valores[1] = aceleracaoY;
  valores[2] = aceleracaoZ;
  valores[3] = giroX - offsetGiroX;
  valores[4] = giroY - offsetGiroY;
  valores[5] = giroZ - offsetGiroZ;
  valores[6] = sqrtf(aceleracaoX * aceleracaoX + aceleracaoY * aceleracaoY + aceleracaoZ * aceleracaoZ);
  for (k = 0; k < 7; k++) {
    est.soma[k] += valores[k];
    est.somaQuadrados[k] += (double)valores[k] * valores[k];
  }
  est.quantidade++;
}

float calcularMedia(Estatistica& est, int k) {
  return est.soma[k] / est.quantidade;
}

// Desvio padrao = raiz( media dos quadrados - quadrado da media ).
float calcularDesvio(Estatistica& est, int k) {
  double media = est.soma[k] / est.quantidade;
  return sqrt(fmax(0.0, est.somaQuadrados[k] / est.quantidade - media * media));
}

// Junta 'alvo' amostras validas (descarta leituras invalidas e conta). Devolve quantas foram descartadas.
int coletarAmostras(Estatistica& est, int alvo) {
  int falhas = 0;

  zerarEstatistica(est);
  while (est.quantidade < alvo && falhas < 5 * alvo) {
    if (lerSensor()) somarLeitura(est); else falhas++;
    delayMicroseconds(PERIODO_US);
  }
  return falhas;
}

// ======================= ETAPA 3: PARADO + CALIBRACAO =======================
// Media de amostras validas com a viga PARADA e NIVELADA.
//   - bias do giro (X,Y,Z)      -> vira offset e e subtraido sempre
//   - roll/pitch em repouso     -> vira o "zero" da viga
//   - ruido = desvio padrao     -> NAO muda com calibracao (e do sensor)
void executarCalibracao() {
  Estatistica janela;
  Estatistica conferencia;
  int falhas = 0;
  int i = 0;

  Serial.println("\nEtapa 3 - CALIBRACAO: viga PARADA e NIVELADA (nao encoste)...");
  for (i = 0; i < AMOSTRAS_AQUECIMENTO; i++) {      // aquece/descarta
    lerSensor();
    delayMicroseconds(PERIODO_US);
  }
  falhas = coletarAmostras(janela, AMOSTRAS_CALIBRACAO);
  offsetGiroX = calcularMedia(janela, 3);           // bias do giro vira offset
  offsetGiroY = calcularMedia(janela, 4);
  offsetGiroZ = calcularMedia(janela, 5);
  aceleracaoX = calcularMedia(janela, 0);
  aceleracaoY = calcularMedia(janela, 1);
  aceleracaoZ = calcularMedia(janela, 2);
  offsetRoll = calcularRollAcel();                  // zero da viga
  offsetPitch = calcularPitchAcel();
  Serial.printf("   amostras validas  : %d  (descartadas: %d)\n", janela.quantidade, falhas);
  Serial.printf("   gravidade medida  : %.4f g (esperado ~1.000)\n", calcularMedia(janela, 6));
  Serial.printf("   ruido do acel.    : X=%.4f Y=%.4f Z=%.4f g (desvio padrao)\n",
                calcularDesvio(janela, 0), calcularDesvio(janela, 1), calcularDesvio(janela, 2));
  Serial.printf("   bias do giro      : X=%.3f Y=%.3f Z=%.3f graus/s  -> virou offset\n", offsetGiroX, offsetGiroY, offsetGiroZ);
  Serial.printf("   ruido do giro     : X=%.4f Y=%.4f Z=%.4f graus/s (desvio padrao)\n",
                calcularDesvio(janela, 3), calcularDesvio(janela, 4), calcularDesvio(janela, 5));
  Serial.printf("   zero da viga      : roll=%.2f pitch=%.2f graus -> virou offset\n", offsetRoll, offsetPitch);
  Serial.printf("   temperatura       : %.1f C\n", temperaturaC);
  mostrarResultado("amostras validas suficientes", janela.quantidade == AMOSTRAS_CALIBRACAO,
                   "muitas leituras invalidas: fio frouxo ou pino errado.");
  mostrarResultado("gravidade entre 0.95 e 1.05 g", calcularMedia(janela, 6) > 0.95 && calcularMedia(janela, 6) < 1.05,
                   "escala errada ou viga se mexendo.");
  mostrarResultado("ruido do giro abaixo de 0.5 graus/s",
                   calcularDesvio(janela, 3) < 0.5 && calcularDesvio(janela, 4) < 0.5 && calcularDesvio(janela, 5) < 0.5,
                   "viga mexeu/vibrou durante a calibracao. Repita ('r').");

  // confere a calibracao com uma 2a janela
  coletarAmostras(conferencia, AMOSTRAS_CONFERENCIA);
  Serial.printf("\n   CONFERENCIA (ja calibrado): bias giro X=%.3f Y=%.3f Z=%.3f graus/s\n",
                calcularMedia(conferencia, 3), calcularMedia(conferencia, 4), calcularMedia(conferencia, 5));
  mostrarResultado("bias residual abaixo de 0.2 graus/s",
                   fabs(calcularMedia(conferencia, 3)) < 0.2 && fabs(calcularMedia(conferencia, 4)) < 0.2
                     && fabs(calcularMedia(conferencia, 5)) < 0.2,
                   "viga mexeu entre as janelas. Repita ('r').");

  // linha CSV padronizada (igual nos testes 1, 2 e 3)
  Serial.println("\n   RESUMO_CSV,sensor,g,rAx,rAy,rAz,bGx,bGy,bGz,rGx,rGy,rGz,temp");
  Serial.printf("   RESUMO_CSV,%s,%.4f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.1f\n", NOME_SENSOR,
                calcularMedia(janela, 6), calcularDesvio(janela, 0), calcularDesvio(janela, 1), calcularDesvio(janela, 2),
                offsetGiroX, offsetGiroY, offsetGiroZ,
                calcularDesvio(janela, 3), calcularDesvio(janela, 4), calcularDesvio(janela, 5), temperaturaC);

  Serial.println("\nEtapa 4 - AO VIVO, JA CALIBRADO (viga nivelada = 0). 'r' + ENTER repete.\n");
}

// ETAPA 4: laco ao vivo (~20 linhas/s). 'r' + ENTER reinicia.
void executarLoopVivo() {
  if (Serial.available() && Serial.read() == 'r') ESP.restart();
  if (lerSensor()) {
    Serial.printf("roll:%.2f pitch:%.2f giroX:%.3f giroY:%.3f giroZ:%.3f\n",
                  calcularRollAcel() - offsetRoll, calcularPitchAcel() - offsetPitch,
                  giroX - offsetGiroX, giroY - offsetGiroY, giroZ - offsetGiroZ);
    delay(50);
  } else {
    delay(5);
  }
}

// ETAPA 1: varredura do barramento. Chama todos os enderecos de 1 a 126 e lista quem responde.
// Devolve true se o MPU6050 (0x68) respondeu.
bool procurarMpu() {
  bool achouMpu = false;
  uint8_t endereco = 0;

  Serial.println("\nEtapa 1 - procurando dispositivos em SDA=D21 / SCL=D22:");
  for (endereco = 1; endereco < 127; endereco++) {
    Wire.beginTransmission(endereco);
    if (Wire.endTransmission() == 0) {                     // 0 = alguem respondeu (ACK)
      Serial.printf("   encontrado no endereco 0x%02X\n", endereco);
      if (endereco == ENDERECO_MPU) achouMpu = true;
    }
  }
  mostrarResultado("MPU6050 responde em 0x68", achouMpu,
                   "confira VCC no 3V3, GND, SDA no D21 e SCL no D22 (nao invertidos).");
  return achouMpu;
}

// ETAPA 2: le o WHO_AM_I. O MPU6050 original devolve 0x68; clones/variantes devolvem
// 0x70, 0x72, 0x98 (MPU6500/6000 remarcados) e continuam funcionando neste teste.
void identificarChip() {
  uint8_t id = 0;
  bool idReconhecido = false;

  lerRegistradores(REG_WHO_AM_I, &id, 1);
  Serial.printf("\nEtapa 2 - WHO_AM_I = 0x%02X\n", id);
  idReconhecido = (id == 0x68 || id == 0x70 || id == 0x72 || id == 0x98);
  mostrarResultado("chip reconhecido", idReconhecido, "valor estranho: modulo falso ou defeituoso.");
  if (id != 0x68 && idReconhecido) {
    Serial.println("        (aviso: nao e um MPU6050 original, mas e compativel)");
  }
}

// Acorda o chip: ao ligar ele vem em modo SLEEP. Escrevendo 0x01 ele sai do sleep
// e usa o oscilador do giroscopio X (mais estavel).
void acordarChip() {
  escreverRegistrador(REG_PWR_MGMT_1, 0x01);
  delay(100);                                              // tempo para o sensor estabilizar
}

void setup() {
  Serial.begin(115200);
  delay(1500);                                             // tempo para abrir o Monitor Serial
  Serial.println("\n===== TESTE 1 - MPU6050 =====");

  Wire.begin(PINO_SDA, PINO_SCL, 400000);                  // I2C a 400 kHz (padrao rapido)

  if (procurarMpu()) {
    identificarChip();
    acordarChip();
    executarCalibracao();                                  // etapa 3; a Etapa 4 roda depois, no loop()
  } else {
    Serial.println("Teste parado.");
    while (true) delay(1000);
  }
}

void loop() {
  executarLoopVivo();                                      // Etapa 4: mostra os angulos ao vivo
}
