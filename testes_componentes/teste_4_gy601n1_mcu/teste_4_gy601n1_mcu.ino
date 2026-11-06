/*
  =====================================================================
   TESTE 4 - MICRO DO GY-601N1 (angulos ja prontos, por UART)
   ESP32 | IC IFSP-PEP | revisado em 04/11/2026
  =====================================================================
   OBJETIVO
     O modulo GY-601N1 tem, alem do BMI323, um microcontrolador que le o
     BMI323, faz a fusao (filtro) e MANDA SOZINHO ROLL, PITCH, YAW e
     temperatura. Este teste so ESCUTA esse micro e mostra se ele responde.
     (O ESP32 nao faz filtro nenhum aqui: o angulo ja vem calculado pelo micro.)

   LIGACAO (so estes fios; os dos testes 2/3 podem continuar ligados)
     VCC    -> D23   (o ESP32 liga/desliga o modulo; NAO vai no 3V3)
     GND    -> GND
     TX/DA  -> D16   (RX2 do ESP32: recebe do micro)
     RX/CL  -> D17   (TX2 do ESP32: so ligado por completeza, nao e usado)
     PS     -> D27   (o ESP32 poe em 3V3 so neste teste = UART; nos outros fica solto)
     CS     -> D25   (ESP32 deixa em 1)
     Soltos: INT1, INT2, INT do micro. Nenhum pino do MPU6050 ou dos ESCs.
   Se for usar em outro projeto: mude so os #define de pinos abaixo.

   COMO O MICRO FALA (confirmado em 03/11/2026)
     PS em 3V3 na hora em que o modulo liga -> UART 9600 baud, 8N1.
     Ele transmite 50 quadros por segundo, 13 bytes cada:
       A4 03 14 08 | roll pitch yaw temp (4 x int16, byte ALTO primeiro, /100) | CHK
     CHK = soma de 8 bits dos 12 primeiros bytes.
     Depois de ligar, o micro leva ~3 s PARADO se calibrando: nao mexa na viga.
     O PS e lido SO quando o modulo liga: por isso o codigo desliga o D23,
     poe o PS em 3V3 e so entao liga o D23 (nao precisa tirar o USB).

   O QUE O TESTE FAZ
     1. Liga o modulo (D23) com PS=3V3 e espera a calibracao do micro.
     2. Escuta 5 s: conta quadros validos, quadros com checksum errado e a taxa.
     3. Mostra OK/FALHOU e, se falhar, o que chegou (bytes crus) para diagnostico.
        Ate 3 tentativas de liga/desliga e, se nada vier, tenta os fios invertidos.
     4. Ao vivo (~10 linhas/s): roll, pitch, yaw, temperatura. 'r' + ENTER repete.
   Monitor Serial 115200.
  =====================================================================
*/
#include <driver/gpio.h>

#define PINO_RX2  16   // <- TX/DA do modulo
#define PINO_TX2  17   // -> RX/CL do modulo
#define PINO_PS   27   // PS do modulo
#define PINO_VCC  23   // VCC do modulo (o ESP32 liga/desliga)
#define PINO_CS   25   // CS do BMI323 (alto: o ESP32 nao usa o chip, so o micro)
#define BAUD_GY   9600

#define TAM_QUADRO       13      // bytes de um quadro: A4 03 14 08 | 4 x int16 | CHK
#define TAM_BUFFER_ESCUTA 6000   // bytes guardados durante a escuta (5 s a 9600 baud cabem folgados)
#define TAM_JANELA_VIVO   200    // bytes lidos entre duas linhas do modo ao vivo

struct Quadro { float roll, pitch, yaw, temp; };

// Estado compartilhado entre setup() e loop()
int  pinoRxAtual = PINO_RX2;
int  pinoTxAtual = PINO_TX2;
bool microEncontrado = false;
uint8_t janelaVivo[TAM_JANELA_VIVO];
int  bytesJanelaVivo = 0;

// Procura quadros A4 03 14 08 ... CHK dentro de 'bytes' (n bytes).
// Cada quadro valido atualiza 'ultimo' e soma 1 no retorno; checksum errado soma em 'invalidos'.
int contarQuadros(const uint8_t* bytes, int n, Quadro& ultimo, int& invalidos) {
  int validos = 0;
  int i = 0;
  int k = 0;
  uint8_t soma = 0;

  while (i + TAM_QUADRO <= n) {
    if (bytes[i] == 0xA4 && bytes[i + 1] == 0x03 && bytes[i + 2] == 0x14 && bytes[i + 3] == 0x08) {
      soma = 0;
      for (k = 0; k < 12; k++) soma += bytes[i + k];
      if (soma == bytes[i + 12]) {
        ultimo.roll  = (int16_t)((bytes[i + 4]  << 8) | bytes[i + 5])  / 100.0f;
        ultimo.pitch = (int16_t)((bytes[i + 6]  << 8) | bytes[i + 7])  / 100.0f;
        ultimo.yaw   = (int16_t)((bytes[i + 8]  << 8) | bytes[i + 9])  / 100.0f;
        ultimo.temp  = (int16_t)((bytes[i + 10] << 8) | bytes[i + 11]) / 100.0f;
        validos++;
        i += TAM_QUADRO;
      } else {
        invalidos++;
        i++;
      }
    } else {
      i++;
    }
  }
  return validos;
}

// Desliga o modulo, poe o PS em 3V3 e CS em 1, liga e espera o micro calibrar.
void ligarModulo() {
  pinMode(PINO_VCC, OUTPUT); digitalWrite(PINO_VCC, LOW);     // modulo desligado
  pinMode(PINO_PS, OUTPUT);  digitalWrite(PINO_PS, LOW);
  pinMode(PINO_CS, OUTPUT);  digitalWrite(PINO_CS, LOW);
  pinMode(18, INPUT); pinMode(19, INPUT); pinMode(26, INPUT);  // SCL/SDA/SA0 soltos: quem fala com o BMI e o micro
  delay(400);                                                  // descarrega
  digitalWrite(PINO_PS, HIGH);                                 // PS = 3V3 ANTES de ligar (modo UART)
  digitalWrite(PINO_CS, HIGH);
  gpio_set_drive_capability((gpio_num_t)PINO_VCC, GPIO_DRIVE_CAP_3);
  digitalWrite(PINO_VCC, HIGH);
  delay(3500);                                                 // micro liga e se calibra (~3 s, viga parada)
}

// Escuta 'duracaoMs' milissegundos num par de pinos e conta os quadros.
// Devolve quantos bytes chegaram; 'brutos' recebe os primeiros bytes para diagnostico.
int escutarUart(int pinoRx, int pinoTx, uint32_t duracaoMs, int& validos, int& invalidos,
                Quadro& ultimo, uint8_t* brutos, int& qtdBrutos) {
  static uint8_t buffer[TAM_BUFFER_ESCUTA];
  int recebidos = 0;
  uint32_t inicio = 0;

  Serial2.end();
  Serial2.setRxBufferSize(1024);
  Serial2.begin(BAUD_GY, SERIAL_8N1, pinoRx, pinoTx);
  delay(30);
  while (Serial2.available()) Serial2.read();

  inicio = millis();
  while (millis() - inicio < duracaoMs) {
    while (Serial2.available() && recebidos < TAM_BUFFER_ESCUTA) buffer[recebidos++] = Serial2.read();
  }

  invalidos = 0;
  qtdBrutos = min(recebidos, 16);
  memcpy(brutos, buffer, qtdBrutos);
  validos = contarQuadros(buffer, recebidos, ultimo, invalidos);
  return recebidos;
}

// Imprime [ OK ] ou [FALHOU] e, se falhou, a dica do que conferir.
void mostrarResultado(const char* etapa, bool passou, const char* dica) {
  Serial.printf("[%s] %s\n", passou ? " OK  " : "FALHOU", etapa);
  if (!passou) Serial.printf("        -> %s\n", dica);
}

void setup() {
  const int pares[2][2] = {{PINO_RX2, PINO_TX2}, {PINO_TX2, PINO_RX2}};   // fios normais e invertidos
  int par = 0;
  int tentativa = 0;
  int bytesRecebidos = 0;
  int validos = 0;
  int invalidos = 0;
  int qtdBrutos = 0;
  int i = 0;
  Quadro ultimo = {0, 0, 0, 0};
  uint8_t brutos[16];

  Serial.begin(115200); delay(1500);
  Serial.println("\n===== TESTE 4 - MICRO DO GY-601N1 (angulos prontos, UART 9600) =====");
  Serial.println("Deixe a viga PARADA ate o fim da calibracao do micro.");

  for (par = 0; par < 2 && !microEncontrado; par++) {
    for (tentativa = 1; tentativa <= 3 && !microEncontrado; tentativa++) {
      Serial.printf("\nEtapa 1 - tentativa %d (%s): VCC=D23, PS=3V3, ESP32 escuta no D%d...\n",
                    tentativa, par ? "fios INVERTIDOS" : "fios normais", pares[par][0]);
      ligarModulo();
      ultimo = {0, 0, 0, 0};
      bytesRecebidos = escutarUart(pares[par][0], pares[par][1], 5000, validos, invalidos, ultimo, brutos, qtdBrutos);
      Serial.printf("   bytes recebidos : %d\n   quadros validos : %d (esperado ~250 em 5 s)\n   checksum errado : %d\n",
                    bytesRecebidos, validos, invalidos);
      if (validos > 0) {
        Serial.printf("   ultimo quadro   : roll=%.2f pitch=%.2f yaw=%.2f temp=%.2f C\n",
                      ultimo.roll, ultimo.pitch, ultimo.yaw, ultimo.temp);
      } else if (bytesRecebidos > 0) {
        Serial.print("   bytes crus      :");
        for (i = 0; i < qtdBrutos; i++) Serial.printf(" %02X", brutos[i]);
        Serial.println();
      }
      if (validos >= 150 && invalidos * 20 <= validos) {   // >= 30 quadros/s e menos de 5 % ruins
        microEncontrado = true;
        pinoRxAtual = pares[par][0];
        pinoTxAtual = pares[par][1];
        mostrarResultado("micro do GY responde (quadros A4 03 14 08 validos, taxa ~50/s)", true, "");
        mostrarResultado("angulos plausiveis (|roll|,|pitch| < 90 graus)",
                         fabs(ultimo.roll) < 90 && fabs(ultimo.pitch) < 90,
                         "angulo absurdo: viga mexeu durante a calibracao do micro. Repita ('r').");
      } else {
        mostrarResultado("micro do GY responde", false,
                         bytesRecebidos == 0
                           ? "nenhum byte: confira VCC em D23, GND, TX/DA em D16 e PS em D27 (ligado, o codigo poe 3V3)."
                           : "chegou lixo/poucos quadros: confira GND comum e fio TX/DA curto.");
      }
    }
  }

  if (microEncontrado) {
    Serial.println("\nEtapa 2 - AO VIVO (~10 linhas/s). 'r' + ENTER repete.\n");
    Serial2.end();
    Serial2.setRxBufferSize(1024);
    Serial2.begin(BAUD_GY, SERIAL_8N1, pinoRxAtual, pinoTxAtual);
  } else {
    Serial.println("\nNenhuma tentativa funcionou. Copie este log.");
  }
}

void loop() {
  static uint32_t ultimoPrintMs = 0;
  Quadro quadro = {0, 0, 0, 0};
  int invalidos = 0;

  if (Serial.available() && Serial.read() == 'r') ESP.restart();

  if (!microEncontrado) {
    delay(100);
  } else {
    while (Serial2.available() && bytesJanelaVivo < TAM_JANELA_VIVO) janelaVivo[bytesJanelaVivo++] = Serial2.read();
    if (millis() - ultimoPrintMs >= 100) {                  // mostra o quadro mais recente 10x/s
      ultimoPrintMs = millis();
      if (contarQuadros(janelaVivo, bytesJanelaVivo, quadro, invalidos) > 0) {
        Serial.printf("roll:%.2f pitch:%.2f yaw:%.2f temp:%.2f\n", quadro.roll, quadro.pitch, quadro.yaw, quadro.temp);
      }
      bytesJanelaVivo = 0;
    }
  }
}
