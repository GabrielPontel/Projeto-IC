/* =====================================================================
   TESTE 3 - BMI323 (GY-601N1) por I2C   |   ESP32  |  IC IFSP-PEP
   =====================================================================
   OBJETIVO
     Ler o BMI323 por I2C com os MESMOS fios do teste 2 (SPI) e fazer as
     mesmas medidas: identidade, configuracao, calibracao parado e ao vivo.

   LIGACAO (mesmos fios do teste 2, sem mudar nada)
     VCC      -> D23   (o ESP32 LIGA/DESLIGA o modulo por este pino; NAO vai no 3V3)
     GND      -> GND
     SCL/SCLK -> D18 (SCL)   SDA/SDI -> D19 (SDA)
     SA0/SDO  -> D26 (o ESP32 poe em 0 -> endereco 0x68)
     CS       -> D25 (o ESP32 poe em 1 e deixa la = modo I2C; fio fixo, nao troca)
     PS       -> SOLTO (nao ligar em nada; micro da placa fica quieto)
     Soltos: INT1, INT2 e, do lado do micro, TX/DA, RX/CL e INT.
     Nenhum pino do MPU6050 ou dos ESCs e usado neste teste.
   PULL-UPS: o codigo liga os pull-ups internos do ESP32 em SDA e SCL e usa
   I2C lento (10 kHz) para funcionar mesmo sem resistor externo. Com 4,7 kohm
   externos em SDA e SCL (para 3V3) pode subir I2C_HZ (100000 ou 400000).
   Monitor Serial 115200.  'r' + ENTER repete.

   PINOS USADOS NESTE CODIGO
     D23 = VCC do modulo (saida; LOW = desligado, HIGH = ligado)
     D19 = SDA   D18 = SCL   D26 = SA0 (saida em 0)   D25 = CS (saida em 1)
     D27 (PS) NAO e usado: fica solto.

   COMO FUNCIONA (resumo)
     - O BMI323 liga em I2C se o CS estiver em ALTO na partida; CS baixo = SPI.
     - No I2C do BMI323, depois do endereco do registrador vem 2 bytes
       FALSOS antes dos dados (por isso lerRegistradores() le 2 + 2*n bytes).
     - Registradores de 16 bits, byte baixo primeiro.
     - ACC_CONF 0x4028 = normal, +-8 g, 100 Hz  (4096 LSB/g)
       GYR_CONF 0x4048 = normal, +-2000 dps, 100 Hz  (16.384 LSB/(graus/s))
     - Dados: ACC 0x03..0x05, GYR 0x06..0x08, TEMP 0x09 (valor/512 + 23).

   STATUS (04/11/2026)
     APROVADO: gravidade ~1,04 g, ruido do giro ~0,05 graus/s, leitura ao vivo estavel.
     PS solto (so entrada), VCC pelo D23 com liga/desliga (CS=1 e SA0=0 antes de ligar:
     o chip nasce em I2C) e pull-ups internos.
   ===================================================================== */
#include <driver/gpio.h>   // forca do pino de alimentacao (D23)
#include <Wire.h>

// ---- pinos do I2C (ver ligacao no cabecalho) ----
#define PINO_SDA  19   // SDA/SDI do modulo
#define PINO_SCL  18   // SCL/SCLK do modulo
#define PINO_SA0  26   // SA0/SDO: em baixo -> endereco 0x68
#define PINO_CS   25   // CS: tem que ficar em ALTO para o chip ficar em I2C
#define PINO_PS   27   // PS: so e posto como entrada (solto)
#define PINO_VCC  23   // VCC do modulo: o ESP32 liga/desliga o modulo por aqui
#define I2C_HZ    10000  // devagar de proposito: so com pull-up interno o sinal sobe devagar (com 4,7 k externos use 100000)

#define NOME_SENSOR "BMI323_I2C"      // nome na linha RESUMO_CSV
#define PERIODO_US 10000              // pausa entre leituras na calibracao (10 ms = 100 Hz)
#define AMOSTRAS_AQUECIMENTO 100      // leituras descartadas antes de calibrar
#define AMOSTRAS_CALIBRACAO 400       // 400 amostras x 10 ms = 4 s
#define AMOSTRAS_CONFERENCIA 200      // segunda janela, para conferir a calibracao
#define ID_BMI323 0x43                // valor esperado do CHIP_ID

const uint8_t ENDERECO_BMI = 0x68;   // endereco I2C do BMI323 (SA0 em baixo)
TwoWire barramento(1);               // segundo controlador I2C do ESP32 (o 0 fica livre)

// Estatistica: somas para media e desvio padrao (soma = soma dos valores,
// somaQuadrados = soma dos quadrados, quantidade = amostras).
// Indice k: 0..2 acel X,Y,Z (g); 3..5 giro X,Y,Z (graus/s sem offset); 6 = modulo da aceleracao (g).
struct Estatistica { double soma[8], somaQuadrados[8]; int quantidade; };   // (no topo: exigencia do Arduino IDE)


const float LSB_POR_G = 4096.0;                  // escala do acelerometro para +-8 g
const float LSB_POR_DPS = 16.384;                // escala do giro para +-2000 graus/s

// Ultima leitura (g, graus/s, graus C)
float aceleracaoX, aceleracaoY, aceleracaoZ;
float giroX, giroY, giroZ;
float temperaturaC;

// Calibracao: o giro em repouso vira offset e a viga nivelada vira o "zero" de roll/pitch.
float offsetGiroX = 0, offsetGiroY = 0, offsetGiroZ = 0;
float offsetRoll = 0, offsetPitch = 0;

bool sensorPronto = false;   // vira true quando o chip foi identificado e calibrado (libera o laco ao vivo)

// BMI323 em I2C: depois do endereco do registrador vem 2 bytes falsos.
void lerRegistradores(uint8_t registrador, uint16_t* destino, uint8_t quantidade) {
  uint8_t recebidos = 0;
  uint8_t i = 0;
  uint8_t baixo = 0;
  uint8_t alto = 0;

  barramento.beginTransmission(ENDERECO_BMI);
  barramento.write(registrador);
  barramento.endTransmission(false);                                            // aponta o registrador (sem STOP)
  recebidos = barramento.requestFrom((uint8_t)ENDERECO_BMI, (uint8_t)(2 + 2 * quantidade));   // 2 falsos + palavras de 2 bytes
  if (recebidos < 2 + 2 * quantidade) {                                         // veio incompleto: zera
    for (i = 0; i < quantidade; i++) destino[i] = 0;
    while (barramento.available()) barramento.read();
  } else {
    barramento.read(); barramento.read();                                       // joga fora os 2 bytes falsos
    for (i = 0; i < quantidade; i++) {
      baixo = barramento.read();                                                // byte baixo primeiro
      alto = barramento.read();
      destino[i] = (uint16_t)(alto << 8 | baixo);
    }
  }
}

// Le 1 registrador de 16 bits.
uint16_t lerRegistrador16(uint8_t registrador) {
  uint16_t valor = 0;
  lerRegistradores(registrador, &valor, 1);
  return valor;
}

// Escreve 16 bits (byte baixo, depois alto).
void escreverRegistrador16(uint8_t registrador, uint16_t valor) {
  barramento.beginTransmission(ENDERECO_BMI);
  barramento.write(registrador);
  barramento.write(valor & 0xFF);
  barramento.write(valor >> 8);
  barramento.endTransmission();
  delayMicroseconds(50);
}

// Escreve e confere lendo de volta (ate 10 tentativas). Devolve true se o chip aceitou.
bool escreverEConferir(uint8_t registrador, uint16_t valor) {
  bool aceitou = false;
  int tentativa = 0;

  for (tentativa = 0; tentativa < 10 && !aceitou; tentativa++) {
    escreverRegistrador16(registrador, valor);
    delay(2);
    if (lerRegistrador16(registrador) == valor) aceitou = true;
  }
  return aceitou;
}

// Le acelerometro, giro e temperatura (2 leituras seguidas: so aceita se baterem).
// Devolve false se as leituras divergem (ruido no barramento) ou o chip mandou 0x8000 (sem dado valido).
bool lerSensor() {
  uint16_t bruto[7];
  uint16_t bruto2[7];
  bool valido = true;
  int i = 0;

  lerRegistradores(0x03, bruto, 7);
  lerRegistradores(0x03, bruto2, 7);
  for (i = 0; i < 6; i++) {
    if (abs((int16_t)bruto[i] - (int16_t)bruto2[i]) > 400) valido = false;   // 2 leituras muito diferentes = ruido
    if (bruto[i] == 0x8000) valido = false;                                   // 0x8000 = BMI323 sem dado valido
  }
  if (valido) {
    aceleracaoX = (int16_t)bruto[0] / LSB_POR_G;
    aceleracaoY = (int16_t)bruto[1] / LSB_POR_G;
    aceleracaoZ = (int16_t)bruto[2] / LSB_POR_G;
    giroX = (int16_t)bruto[3] / LSB_POR_DPS;
    giroY = (int16_t)bruto[4] / LSB_POR_DPS;
    giroZ = (int16_t)bruto[5] / LSB_POR_DPS;
    temperaturaC = (int16_t)bruto[6] / 512.0 + 23.0;
  }
  return valido;
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

// Liga o modulo do jeito certo (VCC pelo D23):
//   1) desliga o VCC e segura todos os fios em baixo (o modulo descarrega de vez);
//   2) poe CS=1 e SA0=0 ANTES de ligar: CS alto na partida = chip em I2C, SA0 baixo = 0x68;
//   3) liga o VCC com a forca maxima do pino e espera o chip iniciar;
//   4) abre o I2C com pull-ups internos. PS nao e tocado (fica solto).
void ligarModulo() {
  const int pinosLinha[4] = {PINO_SDA, PINO_SCL, PINO_SA0, PINO_CS};
  int i = 0;

  barramento.end();                                          // solta os pinos do I2C
  pinMode(PINO_VCC, OUTPUT); digitalWrite(PINO_VCC, LOW);    // modulo desligado
  pinMode(PINO_PS, INPUT);                                   // PS solto (se estiver desconectado, nao faz nada)
  for (i = 0; i < 4; i++) { pinMode(pinosLinha[i], OUTPUT); digitalWrite(pinosLinha[i], LOW); }
  delay(300);                                                // descarrega
  pinMode(PINO_CS, OUTPUT); digitalWrite(PINO_CS, HIGH);     // CS alto = I2C
  pinMode(PINO_SA0, OUTPUT); digitalWrite(PINO_SA0, LOW);    // endereco 0x68
  pinMode(PINO_SDA, INPUT_PULLUP); pinMode(PINO_SCL, INPUT_PULLUP);   // pull-up interno
  gpio_set_drive_capability((gpio_num_t)PINO_VCC, GPIO_DRIVE_CAP_3);  // D23 com mais corrente (modulo ~8 mA)
  digitalWrite(PINO_VCC, HIGH); delay(300);                  // liga e espera o chip iniciar
  barramento.begin(PINO_SDA, PINO_SCL, I2C_HZ);              // agora fala I2C
  barramento.setTimeOut(20);
  delay(10);
}

// Varre os enderecos 1..126 e devolve a lista de quem respondeu, ex.: " 0x68".
String varrerEnderecos() {
  String lista = "";
  char texto[6];
  uint8_t endereco = 0;

  for (endereco = 1; endereco < 127; endereco++) {
    barramento.beginTransmission(endereco);
    if (barramento.endTransmission() == 0) {
      sprintf(texto, " 0x%02X", endereco);
      lista += texto;
    }
  }
  return lista;
}

// ETAPA 0: liga o modulo (D23) e procura o BMI323. Ate 4 tentativas de liga/desliga.
// Em cada uma varre o barramento 3 vezes e so aceita se 0x68 aparecer nas 3 e o CHIP_ID for 0x43.
bool procurarBmi() {
  bool achou = false;
  bool estavel = true;
  bool tem68 = false;
  int tentativa = 0;
  int varredura = 0;
  String lista = "";

  for (tentativa = 1; tentativa <= 4 && !achou; tentativa++) {
    ligarModulo();
    Serial.printf("\nEtapa 0 - tentativa %d: VCC=D23, PS solto, SDA=D19 SCL=D18, %d kHz:\n", tentativa, I2C_HZ / 1000);
    estavel = true;
    for (varredura = 0; varredura < 3; varredura++) {
      lista = varrerEnderecos();
      Serial.printf("   varredura %d:%s\n", varredura + 1, lista.length() ? lista.c_str() : " ninguem respondeu");
      if (lista.indexOf("0x68") < 0) estavel = false;        // 0x68 sumiu = sinal instavel
    }
    tem68 = lista.indexOf("0x68") >= 0;
    Serial.printf("   -> %s\n", estavel ? "0x68 presente nas 3 (fantasmas = sinal fraco, ignorados)" : "0x68 sumiu em alguma varredura");
    if (estavel && tem68 && (lerRegistrador16(0x00) & 0xFF) == ID_BMI323) {
      achou = true;
      Serial.printf("   >>> BMI323 OK na tentativa %d\n", tentativa);
    }
  }
  if (!achou) Serial.println("\n   Nenhuma tentativa funcionou: confira VCC em D23, CS em D25, SA0 em D26, SDA D19, SCL D18, PS SOLTO.");
  return achou;
}

// ETAPA 1: confere o CHIP_ID. Devolve true se for um BMI323.
bool identificarChip() {
  uint16_t id = lerRegistrador16(0x00) & 0xFF;

  Serial.printf("\nEtapa 1 - CHIP_ID = 0x%02X (esperado 0x43)\n", id);
  mostrarResultado("chip e um BMI323", id == ID_BMI323, "I2C nao funcionou neste modulo (ver Etapa 0).");
  return id == ID_BMI323;
}

// ETAPA 2: reseta e configura acelerometro e giro (escreve e confere), e le o registrador de erro.
void configurarSensor() {
  uint16_t confAcc = 0;
  uint16_t confGiro = 0;
  uint16_t erro = 0;

  escreverRegistrador16(0x7E, 0xDEAF);                       // soft reset (CMD)
  delay(5);
  escreverEConferir(0x20, 0x4028);                           // ACC: normal, +-8 g, 100 Hz
  escreverEConferir(0x21, 0x4048);                           // GYR: normal, +-2000 dps, 100 Hz
  delay(100);
  confAcc = lerRegistrador16(0x20);                          // le de volta
  confGiro = lerRegistrador16(0x21);
  erro = lerRegistrador16(0x01);                             // registrador de erro
  Serial.printf("\nEtapa 2 - ACC_CONF=0x%04X GYR_CONF=0x%04X ERR=0x%04X\n", confAcc, confGiro, erro);
  mostrarResultado("configuracao aceita", confAcc == 0x4028 && confGiro == 0x4048 && (erro & 0x01) == 0,
                   "configuracao rejeitada (ERR_REG != 0).");
}

void setup() {
  Serial.begin(115200); delay(1500);
  Serial.println("\n===== TESTE 3 - BMI323 (GY-601N1) por I2C =====");
  procurarBmi();                                             // Etapa 0 (mesmo se falhar, a etapa 1 mostra o motivo)
  if (identificarChip()) {                                   // sem chip nao adianta continuar
    configurarSensor();
    executarCalibracao();                                    // etapa 3 (calibracao); a 4 roda no loop()
    sensorPronto = true;
  }
}

void loop() {
  if (sensorPronto) {
    executarLoopVivo();                                      // Etapa 4: ao vivo
  } else {
    if (Serial.available() && Serial.read() == 'r') ESP.restart();
    delay(100);
  }
}
