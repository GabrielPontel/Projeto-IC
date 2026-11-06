/*
  =====================================================================
   TESTE 5 - ESCs + MOTORES (um lado, depois o outro) + CALIBRACAO
   ESP32  |  IC IFSP-PEP  |  atualizado 02/11/2026 (comentado em 04/11/2026)
   (antes era o "teste 4"; renumerado em 04/11/2026)
  =====================================================================
   OBJETIVO
     Comandar os 2 ESCs / motores BLDC pelo Monitor Serial para:
       - conferir qual motor esta em qual lado e o sentido de giro;
       - calibrar a faixa dos ESCs (1000-2000 us);
       - achar o pulso de PARTIDA de cada motor (para igualar no PID).

   Junta o teste antigo (Projeto Iniciacao cientifica/teste_sentido_motores)
   com o teste novo. ATENCAO: o projeto antique teste go usava ESC1=D33 e ESC2=D32;
   aqui vale a montagem confirmada: ESC1=D32, ESC2=D33.

   *** SEGURANCA ***  TIRE AS HELICES. USB do ESP32 antes da bateria.
   Digite 0 (ou s) + ENTER a qualquer hora para PARAR.

   COMANDOS (Monitor Serial 115200, "Nova linha"):
     1 = liga SO o motor 1 (D32)        2 = liga SO o motor 2 (D33)
     3 = liga os dois                   0 ou s = PARA tudo
     + / - = sobe/desce a velocidade de teste (passo 10 us, max 1300)
     i = status
     x = CALIBRAR faixa dos ESCs (bateria DESLIGADA antes do x)
         -> liga a bateria, espera bipes, digita n
     p = CALIBRAR PARTIDA: sobe devagar SO um motor por vez; voce aperta
         ENTER quando ele comecar a girar. Mostra o pulso de partida de
         cada motor (para igualar os dois no PID).
     e = sequencia automatica: motor 1 por 3 s, pausa, motor 2 por 3 s

   LIGACAO (SO ESTES PINOS): ESC1 sinal D32, ESC2 sinal D33, GND dos ESCs
            no GND do ESP32, fio vermelho fino dos ESCs solto.
            Potencia: bateria + e - nos fios grossos dos ESCs; 3 fios de cada
            ESC no motor. Nenhum pino de sensor e usado.

   PINOS USADOS NESTE CODIGO
     D32 = sinal PWM do ESC 1 (motor 1)    D33 = sinal PWM do ESC 2 (motor 2)

   COMO FUNCIONA O SINAL
     ESC comum le um pulso a cada 20 ms (50 Hz): 1000 us = motor parado/minimo,
     2000 us = maximo. A biblioteca ESP32Servo gera esse pulso por hardware
     (writeMicroseconds). Fora da calibracao o pulso NUNCA passa de 1300 us.
  =====================================================================
*/
#include <ESP32Servo.h>   // gera o PWM de servo/ESC no ESP32 (instale pelo Gerenciador de Bibliotecas)

#define PINO_ESC1 32    // sinal do ESC 1
#define PINO_ESC2 33    // sinal do ESC 2

const int PULSO_MIN = 1000;                  // pulso minimo do ESC (motor parado), em microssegundos
const int PULSO_MAX = 2000;                  // pulso maximo do ESC (so usado na calibracao de faixa)
const int LIMITE_TESTE_US = 1300;            // fora da calibracao nunca passa disso
const int VELOCIDADE_MIN_US = 1040;          // menor velocidade de teste (comando -)
const int PASSO_VELOCIDADE_US = 10;          // quanto + e - mudam a velocidade
const int PULSO_PARTIDA_INICIO_US = 1040;    // onde a calibracao de partida comeca
const int PASSO_PARTIDA_US = 5;              // quanto o pulso sobe a cada passo da calibracao de partida
const int ESPERA_PASSO_PARTIDA_MS = 400;     // tempo parado em cada passo da calibracao de partida

int  velocidadeUs = 1120;                    // pulso de teste atual (muda com + e -)
int  partidaMotor1Us = 0;                    // resultado da calibracao de partida do motor 1
int  partidaMotor2Us = 0;                    // resultado da calibracao de partida do motor 2
bool motor1Ligado = false;
bool motor2Ligado = false;
bool aguardandoN = false;                    // true entre o x e o n da calibracao de faixa
Servo esc1, esc2;

// Manda aos ESCs o pulso atual: a velocidade de teste se o motor esta ligado, senao o minimo.
void aplicarPulsos() {
  esc1.writeMicroseconds(motor1Ligado ? velocidadeUs : PULSO_MIN);
  esc2.writeMicroseconds(motor2Ligado ? velocidadeUs : PULSO_MIN);
}

// PARA tudo: desliga os dois motores e manda o minimo (1000 us).
void pararMotores() {
  motor1Ligado = false;
  motor2Ligado = false;
  aplicarPulsos();
  Serial.println(">> PARADO (1000 us)");
}

// Mostra o estado atual (comando i).
void mostrarStatus() {
  Serial.printf("velocidade=%d us | motor1(D32): %s | motor2(D33): %s\n",
                velocidadeUs, motor1Ligado ? "LIGADO" : "parado", motor2Ligado ? "LIGADO" : "parado");
  if (partidaMotor1Us || partidaMotor2Us) {
    Serial.printf("partida: motor1=%d us  motor2=%d us  diferenca=%d us\n",
                  partidaMotor1Us, partidaMotor2Us, partidaMotor1Us - partidaMotor2Us);
  }
}

// Imprime o menu de comandos.
void mostrarMenu() {
  Serial.println("\n------------------ MENU (SEM HELICES) ------------------");
  Serial.println(" 1 = so motor 1     2 = so motor 2     3 = os dois");
  Serial.println(" 0/s = PARAR        +/- = velocidade   i = status");
  Serial.println(" x = calibrar ESCs  p = calibrar partida  e = um lado, depois o outro");
  Serial.println("--------------------------------------------------------");
  mostrarStatus();
}

// Espera 'duracaoMs'; para tudo se chegar 0/s. Devolve false se foi interrompido.
bool esperarOuParar(unsigned long duracaoMs) {
  unsigned long inicio = millis();
  bool completou = true;
  char tecla = 0;

  while (completou && millis() - inicio < duracaoMs) {
    if (Serial.available()) {
      tecla = Serial.read();
      if (tecla == '0' || tecla == 's' || tecla == 'S') {
        pararMotores();
        completou = false;
      }
    }
    delay(5);
  }
  return completou;
}

// Descarta o que sobrou no buffer da serial (ex.: o ENTER).
void limparSerial() {
  delay(20);
  while (Serial.available()) Serial.read();
}

// --- p: pulso de partida de UM motor ----------------------------------
// Sobe o pulso de 1040 us em passos de 5 us a cada 0,4 s. Quando voce aperta ENTER
// (motor comecou a girar) devolve o pulso daquele instante. '0'/'s' aborta e devolve 0.
// Se chegar em 1300 us sem girar, devolve 0.
int acharPartida(Servo& esc, const char* nomeMotor) {
  int pulsoUs = 0;
  int partidaUs = 0;
  bool encerrou = false;
  unsigned long inicioPasso = 0;
  char tecla = 0;

  Serial.printf("\n%s: subindo 5 us a cada 0,4 s. Aperte ENTER quando GIRAR (0 = parar).\n", nomeMotor);
  limparSerial();

  for (pulsoUs = PULSO_PARTIDA_INICIO_US; pulsoUs <= LIMITE_TESTE_US && !encerrou; pulsoUs += PASSO_PARTIDA_US) {
    esc.writeMicroseconds(pulsoUs);
    Serial.printf("pulso:%d\n", pulsoUs);
    inicioPasso = millis();
    while (!encerrou && millis() - inicioPasso < ESPERA_PASSO_PARTIDA_MS) {
      if (Serial.available()) {
        tecla = Serial.read();
        esc.writeMicroseconds(PULSO_MIN);            // volta ao minimo assim que voce aperta
        if (tecla == '0' || tecla == 's' || tecla == 'S') {
          pararMotores();
          partidaUs = 0;
        } else {
          Serial.printf(">> %s comecou a girar em %d us\n", nomeMotor, pulsoUs);
          limparSerial();
          partidaUs = pulsoUs;
        }
        encerrou = true;
      } else {
        delay(5);
      }
    }
  }

  if (!encerrou) {
    esc.writeMicroseconds(PULSO_MIN);
    Serial.printf(">> %s nao girou ate %d us (confira fios/bateria/calibracao x)\n", nomeMotor, LIMITE_TESTE_US);
    partidaUs = 0;
  }
  return partidaUs;
}

// Comando p: acha a partida do motor 1 e depois do motor 2 e compara.
void calibrarPartida() {
  pararMotores();
  partidaMotor1Us = acharPartida(esc1, "MOTOR 1 (D32)");
  if (esperarOuParar(1500)) {                        // pausa de 1,5 s entre os motores
    partidaMotor2Us = acharPartida(esc2, "MOTOR 2 (D33)");
    Serial.println("\nRESULTADO DA CALIBRACAO DE PARTIDA:");
    mostrarStatus();
    Serial.printf("PARTIDA_CSV,%d,%d\n", partidaMotor1Us, partidaMotor2Us);   // linha CSV para anotar
    Serial.println("Se a diferenca passar de ~20 us, rode o x (calibrar ESCs) e repita.");
  }
}

// --- e: um lado, depois o outro ---------------------------------------
// Motor 1 por 3 s, pausa de 2 s, motor 2 por 3 s (use para ver lado e sentido).
void rodarSequencia() {
  bool continuar = true;

  Serial.println("\nSEQUENCIA: motor 1 por 3 s -> pausa 2 s -> motor 2 por 3 s (0 = parar)");
  motor1Ligado = true; motor2Ligado = false;
  aplicarPulsos();
  Serial.println(">> MOTOR 1 (D32) girando. Qual lado da viga? Sentido?");
  continuar = esperarOuParar(3000);

  if (continuar) {
    pararMotores();
    continuar = esperarOuParar(2000);
  }
  if (continuar) {
    motor1Ligado = false; motor2Ligado = true;
    aplicarPulsos();
    Serial.println(">> MOTOR 2 (D33) girando. Qual lado da viga? Sentido?");
    continuar = esperarOuParar(3000);
  }
  if (continuar) {
    pararMotores();
    Serial.println("Motor errado girou? troque os sinais D32/D33. Sentido errado? troque 2 dos 3 fios motor-ESC.");
  }
}

// Calibracao de faixa, 1a metade (comando x): manda o MAXIMO e espera voce ligar a bateria.
void iniciarCalibracaoFaixa() {
  motor1Ligado = false;
  motor2Ligado = false;
  aguardandoN = true;
  esc1.writeMicroseconds(PULSO_MAX);
  esc2.writeMicroseconds(PULSO_MAX);
  Serial.println("\n=== CALIBRACAO DE FAIXA ===");
  Serial.println(">> MAXIMO (2000 us) no ar. AGORA ligue a bateria e espere os bipes.");
  Serial.println(">> Depois dos bipes digite n.");
}

// Calibracao de faixa, 2a metade (so aceita n): manda o MINIMO e espera os bipes.
void tratarCalibracaoFaixa(char tecla) {
  if (tecla == 'n' || tecla == 'N') {
    esc1.writeMicroseconds(PULSO_MIN);
    esc2.writeMicroseconds(PULSO_MIN);
    Serial.println(">> MINIMO enviado. Espere os bipes de confirmacao...");
    delay(3000);
    aguardandoN = false;
    Serial.println(">> Calibracao de faixa concluida (1000-2000 us).");
    mostrarMenu();
  } else {
    Serial.println(">> Calibrando: depois dos bipes, digite n.");
  }
}

// Executa um comando do menu (1, 2, 3, 0, +, -, i, p, e, x).
void tratarComando(char tecla) {
  bool aplicarNovoEstado = false;            // true so para 1, 2, 3, + e -: reenvia os pulsos

  switch (tecla) {
    case '1':
      motor1Ligado = true; motor2Ligado = false;
      Serial.println(">> So MOTOR 1 (D32).");
      aplicarNovoEstado = true;
      break;
    case '2':
      motor1Ligado = false; motor2Ligado = true;
      Serial.println(">> So MOTOR 2 (D33).");
      aplicarNovoEstado = true;
      break;
    case '3':
      motor1Ligado = true; motor2Ligado = true;
      Serial.println(">> Os DOIS motores.");
      aplicarNovoEstado = true;
      break;
    case '0': case 's': case 'S':
      pararMotores();
      break;
    case '+':
      velocidadeUs = min(LIMITE_TESTE_US, velocidadeUs + PASSO_VELOCIDADE_US);
      Serial.printf(">> velocidade %d us\n", velocidadeUs);
      aplicarNovoEstado = true;
      break;
    case '-':
      velocidadeUs = max(VELOCIDADE_MIN_US, velocidadeUs - PASSO_VELOCIDADE_US);
      Serial.printf(">> velocidade %d us\n", velocidadeUs);
      aplicarNovoEstado = true;
      break;
    case 'i': case 'I':
      mostrarStatus();
      break;
    case 'p': case 'P':
      calibrarPartida();
      mostrarMenu();
      break;
    case 'e': case 'E':
      rodarSequencia();
      mostrarMenu();
      break;
    case 'x': case 'X':
      iniciarCalibracaoFaixa();
      break;
    default:
      break;
  }
  if (aplicarNovoEstado) aplicarPulsos();
}

void setup() {
  Serial.begin(115200); delay(1500);
  Serial.println("\n===== TESTE 5 - ESCs + MOTORES =====");
  Serial.println("!!! TIRE AS HELICES !!!");
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);   // reserva 2 timers do ESP32 para os 2 PWM
  esc1.setPeriodHertz(50); esc2.setPeriodHertz(50);          // pulso a cada 20 ms
  esc1.attach(PINO_ESC1, PULSO_MIN, PULSO_MAX);              // liga o PWM no pino de cada ESC
  esc2.attach(PINO_ESC2, PULSO_MIN, PULSO_MAX);
  Serial.println("Armando: pulso minimo por 3 s...");
  pararMotores(); delay(3000);                               // ESC "arma" ao receber 1000 us por alguns segundos
  Serial.println("PRONTO.");
  mostrarMenu();
}

void loop() {
  char tecla = 0;

  if (Serial.available()) {
    tecla = Serial.read();
    if (tecla == '\n' || tecla == '\r') {
      // ignora o ENTER
    } else if (aguardandoN) {
      tratarCalibracaoFaixa(tecla);             // entre x e n so aceita n
    } else {
      tratarComando(tecla);
    }
  }
}
