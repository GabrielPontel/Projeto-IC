/*
  =====================================================================
   TESTE 2 (arquivo 2B) - BOSCH BMI323 (chip real do modulo GY-601N1)
   POR SPI   |   ESP32  |  IC IFSP-PEP
  =====================================================================
   OBJETIVO
     Ler o BMI323 e conferir se o sinal esta bom:
       1. Identidade : CHIP_ID (reg 0x00) tem que ser 0x43
       2. Configura  : acelerometro +-8 g e giroscopio +-2000 dps, 100 Hz
       3. Parado 2 s : gravidade, ruido e bias do giroscopio
       4. Ao vivo    : roll/pitch e temperatura

   LIGACAO (a mesma em que o teste 5 antigo identificou o chip)
     VCC -> D23     GND -> GND     (VCC NAO vai no 3V3: o ESP32 liga/desliga o modulo por D23)
     PS  -> SOLTO   (nao ligar em nada; micro da placa fica quieto)
     SCL/SCLK -> D18  (clock SPI)
     SDA/SDI  -> D19  (MOSI: ESP32 -> sensor)
     SA0/SDO  -> D26  (MISO: sensor -> ESP32)
     CS       -> D25
   Monitor Serial 115200.  'r' + ENTER repete.
   Soltos: INT1, INT2 e, do lado do micro, TX/DA, RX/CL e INT.
   Nenhum pino do MPU6050 ou dos ESCs e usado neste teste.

   PINOS USADOS NESTE CODIGO
     D23 = VCC do modulo (saida)   D18 = SCK   D19 = MOSI   D26 = MISO   D25 = CS (fixo em 1 fora das leituras)
     D27 (PS) NAO e usado: fica solto.

   COMO O BMI323 FUNCIONA (Bosch BST-BMI323-DS000)
     - Registradores de 16 bits, byte BAIXO primeiro (little-endian).
     - Leitura SPI: envia (reg | 0x80), recebe 1 byte "falso" e depois
       os dados.  Escrita SPI: envia reg, byte baixo, byte alto.
     - Liga em I2C; a 1a leitura SPI (com CS em baixo) troca para SPI.
     - ACC_CONF 0x20 / GYR_CONF 0x21: modo[14:12], range[6:4], odr[3:0]
         modo normal = 0b100 ; odr 100 Hz = 0x8
         acel range 0b010 = +-8 g   -> 4096 LSB/g
         giro range 0b100 = +-2000 dps -> 16.384 LSB/(graus/s)
     - Dados: ACC X/Y/Z = 0x03..0x05, GYR X/Y/Z = 0x06..0x08,
       TEMP = 0x09 (graus C = valor/512 + 23).

   STATUS (04/11/2026)
     APROVADO: gravidade ~1,04 g, ruido do giro ~0,05 graus/s, leitura ao vivo estavel.
     O PS (D27) fica solto (aqui e so entrada) e o VCC vai pelo D23, ligado so depois de CS=1.

   REFERENCIAS
     - Bosch Sensortec, BMI323 Datasheet BST-BMI323-DS000
     - https://github.com/DFRobot/DFRobot_BMI323
     - https://github.com/nopnop2002/esp-idf-bmi323
  =====================================================================
*/
#include <SPI.h>            // biblioteca SPI do ESP32
#include <driver/gpio.h>    // forca do pino de alimentacao (D23)

// Estatistica: somas para media e desvio padrao (soma = soma dos valores,
// somaQuadrados = soma dos quadrados, quantidade = amostras).
// Indice k: 0..2 acel X,Y,Z (g); 3..5 giro X,Y,Z (graus/s sem offset); 6 = modulo da aceleracao (g).
struct Estatistica { double soma[8], somaQuadrados[8]; int quantidade; };   // (no topo: exigencia do Arduino IDE)

// ---- pinos do SPI (ver ligacao no cabecalho) ----
#define PINO_SCK  18    // SCL/SCLK do modulo
#define PINO_MOSI 19    // SDA/SDI do modulo
#define PINO_MISO 26    // SA0/SDO do modulo
#define PINO_CS   25    // CS do modulo
#define PINO_VCC  23    // VCC do modulo (o ESP32 liga/desliga)
#define PINO_PS   27    // PS: so e posto como entrada (solto)

#define NOME_SENSOR "BMI323_SPI"      // nome na linha RESUMO_CSV
#define PERIODO_US 10000              // pausa entre leituras na calibracao (10 ms = 100 Hz)
#define AMOSTRAS_AQUECIMENTO 100      // leituras descartadas antes de calibrar
#define AMOSTRAS_CALIBRACAO 400       // 400 amostras x 10 ms = 4 s
#define AMOSTRAS_CONFERENCIA 200      // segunda janela, para conferir a calibracao
#define ID_BMI323 0x43                // valor esperado do CHIP_ID

SPIClass spi(HSPI);                              // usa o barramento HSPI do ESP32
SPISettings configSpi(1000000, MSBFIRST, SPI_MODE0);   // 1 MHz: tolera jumpers

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

// Le 'quantidade' palavras de 16 bits a partir de 'registrador'.
void lerRegistradores(uint8_t registrador, uint16_t* destino, uint8_t quantidade) {
  uint8_t i = 0;
  uint8_t baixo = 0;
  uint8_t alto = 0;

  spi.beginTransaction(configSpi);
  digitalWrite(PINO_CS, LOW);                      // CS baixo = comeca a conversa
  spi.transfer(registrador | 0x80);                // bit 7 = 1 significa LEITURA
  spi.transfer(0);                                 // byte falso do BMI323
  for (i = 0; i < quantidade; i++) {
    baixo = spi.transfer(0);                       // byte baixo primeiro
    alto = spi.transfer(0);
    destino[i] = (uint16_t)(alto << 8 | baixo);
  }
  digitalWrite(PINO_CS, HIGH);                     // CS alto = fim
  spi.endTransaction();
}

// Le 1 registrador de 16 bits.
uint16_t lerRegistrador16(uint8_t registrador) {
  uint16_t valor = 0;
  lerRegistradores(registrador, &valor, 1);
  return valor;
}

// Escreve 16 bits em 'registrador' (byte baixo, depois alto).
void escreverRegistrador16(uint8_t registrador, uint16_t valor) {
  spi.beginTransaction(configSpi);
  digitalWrite(PINO_CS, LOW);
  spi.transfer(registrador & 0x7F);                // bit 7 = 0 = ESCRITA
  spi.transfer(valor & 0xFF);
  spi.transfer(valor >> 8);
  digitalWrite(PINO_CS, HIGH);
  spi.endTransaction();
  delayMicroseconds(50);                           // o chip precisa de um tempo para aceitar
}

// Le acelerometro, giro e temperatura. Devolve false se o chip mandou 0x8000 (sem dado valido).
bool lerSensor() {
  uint16_t bruto[7];
  bool valido = true;
  int i = 0;

  lerRegistradores(0x03, bruto, 7);                // acc x,y,z, gyr x,y,z, temp
  for (i = 0; i < 6; i++) {
    if (bruto[i] == 0x8000) valido = false;
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

// Liga o modulo do jeito certo: desliga o VCC (D23), CS em alto, PS solto, depois liga.
void ligarModulo() {
  pinMode(PINO_VCC, OUTPUT); digitalWrite(PINO_VCC, LOW);  // modulo desligado
  pinMode(PINO_PS, INPUT);                                 // PS solto
  pinMode(PINO_CS, OUTPUT); digitalWrite(PINO_CS, HIGH);   // CS em alto = chip nao selecionado
  delay(300);                                              // descarrega
  gpio_set_drive_capability((gpio_num_t)PINO_VCC, GPIO_DRIVE_CAP_3);   // D23 com mais corrente (~8 mA do modulo)
  digitalWrite(PINO_VCC, HIGH); delay(300);                // liga e espera o chip iniciar
  spi.begin(PINO_SCK, PINO_MISO, PINO_MOSI, PINO_CS);      // liga o HSPI nos pinos escolhidos
  delay(10);
  lerRegistrador16(0x00);                                  // 1a leitura: ativa o modo SPI (resultado descartado)
  delay(5);
}

// ETAPA 1: confere o CHIP_ID. Devolve true se for um BMI323.
bool identificarChip() {
  uint16_t id = lerRegistrador16(0x00) & 0xFF;             // byte baixo do CHIP_ID

  Serial.printf("\nEtapa 1 - CHIP_ID = 0x%02X (esperado 0x43)\n", id);
  mostrarResultado("chip e um BMI323", id == ID_BMI323, "sem resposta: confira CS D25, SCLK D18, MOSI D19, MISO D26.");
  return id == ID_BMI323;
}

// ETAPA 2: reseta e configura acelerometro e giro, e confere o que o chip aceitou.
void configurarSensor() {
  uint16_t confAcc = 0;
  uint16_t confGiro = 0;
  uint16_t erro = 0;

  escreverRegistrador16(0x7E, 0xDEAF);                     // soft reset (CMD)
  delay(5);
  lerRegistrador16(0x00);                                  // reativa SPI depois do reset
  escreverRegistrador16(0x20, 0x4028);                     // ACC: normal, +-8 g, 100 Hz
  escreverRegistrador16(0x21, 0x4048);                     // GYR: normal, +-2000 dps, 100 Hz
  delay(100);
  confAcc = lerRegistrador16(0x20);                        // le de volta
  confGiro = lerRegistrador16(0x21);
  erro = lerRegistrador16(0x01);                           // registrador de erro
  Serial.printf("\nEtapa 2 - ACC_CONF=0x%04X GYR_CONF=0x%04X ERR=0x%04X\n", confAcc, confGiro, erro);
  mostrarResultado("configuracao aceita", confAcc == 0x4028 && confGiro == 0x4048 && (erro & 0x01) == 0,
                   "configuracao rejeitada (ERR_REG != 0).");
}

void setup() {
  Serial.begin(115200); delay(1500);
  Serial.println("\n===== TESTE 2 - BMI323 (GY-601N1) por SPI =====");
  ligarModulo();
  if (identificarChip()) {                                 // sem chip nao adianta continuar
    configurarSensor();
    executarCalibracao();                                  // etapa 3 (calibracao); a 4 roda no loop()
    sensorPronto = true;
  }
}

void loop() {
  if (sensorPronto) {
    executarLoopVivo();                                    // Etapa 4: ao vivo
  } else {
    if (Serial.available() && Serial.read() == 'r') ESP.restart();
    delay(100);
  }
}
