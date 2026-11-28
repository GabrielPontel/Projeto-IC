//Iniciação Científica (PIBIFSP) - IFSP Presidente Epitácio
//Estabilização de um Pêndulo com Dois Motores Brushless e Controle PID via ESP32
//Autor: Gabriel Pontel de Mori
//Orientador: Prof. Dr. Ricardo C. C. Ferrari
//
//Código de demonstração: sensor BMI323 com filtro interno (UART), 2 motores, PID Kp 0,31 / Ki 0,4 / Kd 0,14 (PID final da busca em buscar_PID/).
//Ao ligar, calibra o sensor e o acelerador sobe sozinho; o terminal mostra roll e pitch.
//
//Créditos: base no dRehmFlight, de Nicholas Rehm (GPL-3.0), github.com/nickrehm/dRehmFlight;
//filtro de Madgwick adaptado de github.com/arduino-libraries/MadgwickAHRS.



//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
//O receptor PWM fica desativado (radioSetup() com #if 0): nao ha receptor de radio e os pinos
//padrao (21 e 22) sao os mesmos do I2C. Os comandos vem pela serial, no fim de cada loop() (pollSerialCommands()).
#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default (mantido so para compilar a biblioteca; NAO e usado quando USE_BMI323_SPI esta ligado)
//BMI323 por SPI (modulo GY-601N1, lado do chip, dado bruto). 1 = BMI323 SPI, 0 = MPU6050 I2C
#define USE_BMI323_SPI 0
//GY-601N1 pela UART (angulo ja filtrado pelo micro da placa). 1 = ligado. Use SO UM dos dois (BMI323_SPI ou GY_UART).
#define USE_GY_UART 1
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//



//REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <ESP32Servo.h> //Biblioteca de PWM para os ESCs.
//Semantica verificada na fonte da ESP32Servo (ESP32Servo.cpp, Servo::write):
//  - value < 500  -> tratado como ANGULO (0-180), mapeado para [min,max] do attach()
//  - value >= 500 -> tratado como largura em MICROSEGUNDOS
//Como scaleCommands() produz sX_command_PWM no intervalo 0-180 (angulos) e o setup usa
//write(0) (= 900us com attach(pin,900,2100)), as chamadas .write() originais funcionam
//com a MESMA semantica do PWMServo/Arduino Servo - NAO ha necessidade de writeMicroseconds().

#if defined USE_SBUS_RX
  #include "src/SBUS/SBUS.h"   //sBus interface
#endif

#if defined USE_DSM_RX
  #include "src/DSMRX/DSMRX.h"  
#endif

#if defined USE_MPU6050_I2C
  #include "src/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "src/MPU9250/MPU9250.h"
  MPU9250 mpu9250(SPI2,36);
#else
  #error No MPU defined... 
#endif



//========================================================================================================================//



//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.31;   //Roll P-gain - angle mode 
float Ki_roll_angle = 0.4;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.14;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.31;  //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.4;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.14;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)



//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//NOTE: pinos remapeados para o ESP32 DevKit. NAO usar GPIO 6-11 (ligados a flash).
//Radio:
//NOTA: os pinos chXPin/PPM_Pin abaixo sao mantidos apenas para o codigo compilar;
//o receptor PWM esta DESATIVADO (sem attachInterrupt - ch5=21/ch6=22 conflitariam com o I2C).
const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut) - CONFLITA com SDA (21): interrupts desativados
const int ch6Pin = 22; //aux1 - CONFLITA com SCL (22): interrupts desativados
const int PPM_Pin = 23;
//Pinos de saida dos ESCs:
//m3 e m4 usam os pinos 32 e 33 (trocados em relacao ao dRehmFlight original, conforme a fiacao da bancada).
//===== ADAPTACAO 2 MOTORES =====
//ESC1 sinal -> D32 (motor ESQUERDO) | ESC2 sinal -> D33 (motor DIREITO)
//Se a viga corrigir para o lado ERRADO (acelera o motor do lado que ja esta baixo),
//troque INVERTER_LADO para 1 (ou inverta os fios de sinal D32/D33).
//Se o MPU6050 estiver montado de modo que a inclinacao da viga apareca em PITCH
//(e nao em ROLL), troque USAR_EIXO_PITCH para 1.
#define INVERTER_LADO   1
#define USAR_EIXO_PITCH 0
//CALIBRAR_ESC 1 = modo de calibracao dos ESCs pelo Monitor Serial (SEM HELICE!).
//Grave com 1, calibre, depois volte para 0 e grave de novo.
#define CALIBRAR_ESC    0
const int m1Pin = 32; //ESC1 (D32) = motor ESQUERDO
const int m2Pin = 33; //ESC2 (D33) = motor DIREITO
const int m3Pin = -1; //2 motores: m3 NAO usado (sem pino)
const int m4Pin = -1; //2 motores: m4 NAO usado (sem pino)
const int m5Pin = 25; //livre/seguro (nao usado)
const int m6Pin = 26; //livre/seguro (nao usado)
//PWM servo or ESC outputs:
const int servo1Pin = 13; //13/14/15/16 sao GPIOs validos para LEDC (6-11 sao pins de flash e 12 e strapping)
const int servo2Pin = 14; 
const int servo3Pin = 15;
const int servo4Pin = 16;
//servo5-7: NAO anexados (nao ha saidas servo extras; anexar so desperdicaria
//canais LEDC). Os .write() de servo nao anexado sao NO-OP SEGURO (ESP32Servo.cpp:
//writeTicks() so grava se this->attached() for true).
//servo5-7: criados mas NAO anexados (nao ha saidas servo extras; anexar so
//desperdicaria canais LEDC). Os .write() em servo nao anexado sao NO-OP SEGURO
//(ESP32Servo.cpp: writeTicks() so grava se this->attached() for true).
Servo servo1;  //Create servo objects to control a servo or ESC with PWM (ESP32Servo)
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;  //nao anexado - write() e no-op
Servo servo6;  //nao anexado - write() e no-op
Servo servo7;  //nao anexado - write() e no-op

//ESCs: PWM de servo 400 Hz, 1000-2000 us (o OneShot125 original nao e suportado pelos ESCs
//SimonK HW-30A; ver commandMotors() e scaleCommands())
Servo esc1, esc2, esc3, esc4;
constexpr int ESC_FREQ_HZ = 400; //Hz
//LED do DevKit e instrumentacao de loop
const int ledPin = 2; //LED onboard tipico dos ESP32 DevKit



//========================================================================================================================//



//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

#if defined USE_SBUS_RX
  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
  DSM1024 DSM;
#endif

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//Flight status
bool armedFly = false;

//========================================================================================================================//
//                                    INSTRUMENTACAO (jitter de loop + vibracao)                                     //
//========================================================================================================================//
//Definicoes usadas nas medicoes:
// - bodyUs: tempo de trabalho do ciclo = micros() no fim do corpo do loop, ANTES da espera do loopRate().
// - periodUs: distancia entre os INICIOS de dois ciclos consecutivos (inclui a espera do loopRate).
// - estouro: bodyUs >= 4000 us (o loop passou de 4 ms).
// - vibRms: media movel exponencial (alpha = 0.01 por ciclo @2kHz, tau ~ 50 ms) de
//   | |a| - 1g |, com a = acelerafiltrada ja escalada em g (mesma que entra no Madgwick).
// - vibPeak: maior desvio | |a| - 1g | da janela de 1 s (zerado a cada impressao).
const unsigned long LOOP_TARGET_US = 500; //alvo do loopRate(2000) em us
const unsigned long ESTOURO_LIMITE_US = 4000; //estouro: ciclo >= 4 ms
//Acumuladores da janela de 1 s (zerados a cada impressao de STATS):
unsigned long statsBodySum = 0, statsBodyMax = 0, statsEstouros = 0, statsCiclos = 0;
//log rapido opcional (tecla 'l') p/ graficos de resposta - ~100 Hz
bool logFast = false;
int logFastN = 0;
unsigned long statsPerSum = 0, statsPerMax = 0, statsPerMin = 0xFFFFFFFF;
unsigned long statsLastLoopStartUs = 0; //base p/ periodUs
unsigned long statsPrintTimerMs = 0;     // millis() da ultima impressao STATS
bool statsPrimed = false;               //descarta o primeiro periodUs (contem a espera do setup)
//Vibracao:
float vibRms = 0.0f;
float vibPeak = 0.0f;
//Comando serial substituto do receptor (receptor PWM desativado neste port):
unsigned long serThrottle = 1000; //vira channel_1_pwm (throttle); 1000 = minimo
bool modoCalibraESC = false;
//AJUSTE AO VIVO (2 motores): limite de acelerador e desarme por angulo
unsigned long limiteThr = 1400;    //acelerador maximo permitido (#L muda), protege nos testes
float anguloMaxDesarme = 45.0;     //|angulo| acima disso desarma sozinho (#A muda)       //true so dentro de calibrateESCs()
bool serCut = false;              //false = NAO cortado (permite armar); true = corte (desarma)
//DEMO: acelerador automatico (sem comandos no terminal)
const unsigned long THR_DEMO = 1150; //acelerador de trabalho, o mesmo dos testes
unsigned long autoT0 = 0, autoRampMs = 0;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(ledPin, OUTPUT); //LED onboard
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  //pinMode(m3Pin, OUTPUT); //2 motores: nao usado (25/26 sao do BMI323 na sua montagem)
  //pinMode(m4Pin, OUTPUT); //2 motores: nao usado (25/26 sao do BMI323 na sua montagem)
  //pinMode(m5Pin, OUTPUT); //2 motores: nao usado (25/26 sao do BMI323 na sua montagem)
  //pinMode(m6Pin, OUTPUT); //2 motores: nao usado (25/26 sao do BMI323 na sua montagem)
  //anexa apenas servo1-4 (servo5-7 ficam desanexados; write() deles e no-op)
  //servo1.attach(servo1Pin, 900, 2100); //2 motores: desativado (13/14/16 usados por outros fios) //Pin, min PWM value, max PWM value
  //servo2.attach(servo2Pin, 900, 2100); //2 motores: desativado (13/14/16 usados por outros fios)
  //servo3.attach(servo3Pin, 900, 2100); //2 motores: desativado (13/14/16 usados por outros fios)
  //servo4.attach(servo4Pin, 900, 2100); //2 motores: desativado (13/14/16 usados por outros fios)
  // ESCs (SimonK HW-30A): PWM 400 Hz / 1000-2000 us nos pinos m1-m4
  esc1.setPeriodHertz(ESC_FREQ_HZ); esc1.attach(m1Pin, 1000, 2000);
  esc2.setPeriodHertz(ESC_FREQ_HZ); esc2.attach(m2Pin, 1000, 2000);
  //esc3: nao existe na versao 2 motores
  //esc4: nao existe na versao 2 motores
  //servo5.attach(servo5Pin, 900, 2100); //nao usado neste setup ESP32
  //servo6.attach(servo6Pin, 900, 2100);
  //servo7.attach(servo7Pin, 900, 2100);

  //Set built in LED to turn on to signal startup
  digitalWrite(ledPin, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //estado inicial do comando serial que substitui o receptor.
  //serCut=false => ch5=1000 (NAO cortado); serThrottle=1000 (throttle minimo).
  //Com o throttle em 1000 e ch5 em 1000, armedStatus() arma com motores no minimo (seguro).
  serThrottle = 1000;
  serCut = false;
  statsPrintTimerMs = millis();
  Serial.println("Pendulo com 2 motores - ESP32 [IMU: GY-601N1 UART, angulo pronto do micro] (ESC1=D32 esq, ESC2=D33 dir)");
  Serial.println("DEMO: mantenha a haste PARADA e NIVELADA. O acelerador sobe sozinho 5 s depois da calibracao.");
  Serial.println("  Para parar: envie a letra d no monitor serial.");

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //Calibracao automatica a cada ligacao. Exige a haste PARADA e NIVELADA ao ligar (~5 s):
//zera o vies do giroscopio e do acelerometro.
//Se mexer durante a calibracao: reinicie ou tecle 'c' depois (desarmado).
  calculate_IMU_error();

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  
  delay(5);

#if CALIBRAR_ESC
  calibrateESCs(); //modo calibracao ESC via serial (nao sai daqui)
#endif
  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //Arm ESCs (PWM de servo: minimo = 1000 us; no original OneShot125 era 125)
  m1_command_PWM = 1000;
  m2_command_PWM = 1000;
  m3_command_PWM = 1000;
  m4_command_PWM = 1000;
  m5_command_PWM = 1000;
  m6_command_PWM = 1000;
  armMotors(); //Loop over commandMotors() until ESCs happily arm
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
  autoT0 = millis(); //DEMO: a contagem de 5 s comeca aqui

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {
  //===== inicio da medicao do ciclo =====
  unsigned long t0 = micros(); //marcador de inicio de ciclo
  unsigned long periodUs = t0 - statsLastLoopStartUs; //distancia entre inicios de ciclo consecutivos
  unsigned long bodyUs = 0;   //tempo de trabalho do corpo do loop (medido antes do loopRate)
  statsLastLoopStartUs = t0;
  if (statsPrimed) { //o primeiro periodUs (logo apos o setup) e descartado
    statsPerSum += periodUs;
    if (periodUs > statsPerMax) statsPerMax = periodUs;
    if (periodUs < statsPerMin) statsPerMin = periodUs;
  }
  statsPrimed = true;

  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  //printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  //printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

  // Get arming status
  armedStatus(); //Check if the throttle cut is off and throttle is low.

  //Get vehicle state
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits
  
  //PID Controller - SELECT ONE:
  controlANGLE(); //Stabilize on angle setpoint
  //controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  //controlRATE(); //Stabilize on rate setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Seguranca (2 motores): viga inclinou demais -> corta e desarma
#if USAR_EIXO_PITCH
  if (armedFly && fabs(pitch_IMU) > anguloMaxDesarme) {
#else
  if (armedFly && fabs(roll_IMU) > anguloMaxDesarme) {
#endif
    serCut = true; channel_5_pwm = 2000;
    Serial.println("SEGURANCA: angulo passou do limite -> DESARMADO");
  }
#if USE_GY_UART
  if (armedFly && gyIdadeMs() > 300) {   //sensor parou de mandar quadros -> nao voar as cegas
    serCut = true; channel_5_pwm = 2000;
    Serial.println("SEGURANCA: GY-601N1 sem quadros ha >300 ms -> DESARMADO");
  }
#endif

  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); //Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
  servo7.write(s7_command_PWM);

  //===== log rapido opcional (tecla 'l') para GRAFICOS de resposta =====
  //~100 Hz (1 em 10 ciclos; loop ~1067 Hz). Campos:
  //LOG,<ms>,roll,pitch,gx,gy,gz,pidRoll,pitch_PID,pidYaw,m1,m2,m3,m4 (PWM 1000-2000).
  //NOTA: nao usar durante a janela de medicao de JITTER (o printf soma ~30-50 us ao corpo
  //em 1 de cada 10 ciclos); para metricas, rode com o log DESLIGADO.
  if (logFast) {
    if (++logFastN >= 10) {
      logFastN = 0;
      Serial.printf("LOG,%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%.4f,%.4f,%.4f,%lu,%.4f,%.4f,%.4f,%lu\n",
                    (unsigned long)(micros()/1000),
                    (double)roll_IMU, (double)pitch_IMU,
                    (double)GyroX, (double)GyroY, (double)GyroZ,
                    (double)roll_PID, (double)pitch_PID, (double)yaw_PID,
                    (int)m1_command_PWM, (int)m2_command_PWM,
                    (int)m3_command_PWM, (int)m4_command_PWM,
                    Kp_roll_angle, Ki_roll_angle, Kd_roll_angle, channel_1_pwm,
                    (double)AccX, (double)AccY, (double)AccZ, (unsigned long)micros());
    }
  }

  //Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //===== comando serial substitui o receptor =====
  //Aplicado DEPOIS de getCommands()+failSafe() e ANTES de armedStatus()/getDesState() do
  //proximo ciclo 
  //failSafe() nao tem mais efeito sobre os canais usados: este bloco REESCREVE ch1..ch5
  //a cada ciclo, entao valores "garbage" do RX desativado sao sempre sobrescritos.
  pollSerialCommands(); //processa teclas 'a','d','+','-','p' pendentes
  //DEMO: depois de 5 s sobe sozinho o acelerador ate THR_DEMO (25 us a cada 150 ms)
  if (serCut) { serThrottle = 1000; }
  else if (millis() - autoT0 > 5000 && serThrottle < THR_DEMO && millis() - autoRampMs > 150) {
    autoRampMs = millis();
    serThrottle += 25;
    if (serThrottle > THR_DEMO) serThrottle = THR_DEMO;
  }
  channel_1_pwm = serThrottle;      //ch1 = throttle (armedStatus exige <1050 para armar)
  channel_2_pwm = 1500;             //ch2 = roll centrado
  channel_3_pwm = 1500;             //ch3 = pitch centrado
  channel_4_pwm = 1500;             //ch4 = yaw centrado
  channel_5_pwm = serCut ? 2000 : 1000; //ch5 = gear/corte: >1500 corta e desarma; 1000 = liberado

  //===== fim da medicao do ciclo + janela de 1 s =====
  bodyUs = micros() - t0; //trabalho do ciclo ANTES da espera do loopRate
  statsBodySum += bodyUs;
  if (bodyUs > statsBodyMax) statsBodyMax = bodyUs;
  if (bodyUs >= ESTOURO_LIMITE_US) statsEstouros++; //estouro = bodyUs >= 4000 us (trava de 4 ms)
  statsCiclos++;
  printStatsIfDue(); //imprime e zera a janela STATS uma vez por segundo

  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//



void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
   
  //Quad mixing - EXAMPLE
  //===== MIXER 2 MOTORES (viga, empuxo diferencial em 1 eixo) =====
  //Mesma convencao do quad original: lado esquerdo recebe +roll_PID, direito -roll_PID.
  //Pitch e yaw nao tem como ser controlados com 2 motores fixos: sao ignorados.
#if USAR_EIXO_PITCH
  float eixo_PID = -pitch_PID; //IMU girada 90 graus: inclinacao da viga aparece em pitch
#else
  float eixo_PID = roll_PID;
#endif
#if INVERTER_LADO
  eixo_PID = -eixo_PID;
#endif
  m1_command_scaled = thro_des + eixo_PID; //ESC1 D32 - motor ESQUERDO
  m2_command_scaled = thro_des - eixo_PID; //ESC2 D33 - motor DIREITO
  m3_command_scaled = 0; //nao usado
  m4_command_scaled = 0; //nao usado
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;
 
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
    armedFly = true;
  }
}


//=====================================================================================
// GY-601N1 pela UART (ANGULO PRONTO do micro da placa)
// Descoberto em teste (03/11/2026):
//   PS = 3V3 -> o micro da placa le o BMI323 e manda por UART 9600 8N1 quadros de 13 bytes:
//     A4 03 14 08 | roll(2) pitch(2) yaw(2) temp(2) | CHK     (int16 /100, byte alto primeiro)
//     CHK = soma dos 12 bytes anteriores (8 bits).  Apos enviar A5 52 a taxa sobe para ~50 quadros/s.
//   O micro se autocalibra ~3 s apos ligar (viga PARADA). O filtro/algoritmo dele e DESCONHECIDO.
// Pinos: TX/DA do GY -> D16 (RX2) | RX/CL -> D17 (TX2) | PS -> D27 | VCC -> D23 | CS D25 | SA0 D26
//        (SCL D18 e SDA D19 ficam em alta impedancia: so o micro conversa com o BMI323)
// Aqui NAO ha giro nem Madgwick: roll_IMU vem direto do micro (menos a "tara" do boot) e a
// velocidade angular (GyroX, usada no termo D do controlANGLE) e DERIVADA do proprio angulo.
//=====================================================================================
#if USE_GY_UART
  #define GY_PINO_VCC 23
  #define GY_PINO_PS  27
  #define GY_PINO_CS  25
  #define GY_PINO_SA0 26
  #define GY_PINO_SCL 18
  #define GY_PINO_SDA 19
  #define GY_PINO_RX  16   //<- TX/DA do micro
  #define GY_PINO_TX  17   //-> RX/CL do micro
  //Qual campo do quadro e a inclinacao da viga (0 = "roll", 1 = "pitch") e o sinal.
  //Ajustar no teste de repouso: viga com motor 2 para baixo => roll_IMU NEGATIVO (igual MPU/BMI).
  #define GY_CAMPO_ROLL 1
  #define GY_SGN_ROLL   1
  #define GY_FILTRO_VEL 0.4f      //passa-baixa da velocidade derivada (0..1; maior = menos filtro)
  //===== VERSAO TURBO (17/11/2026) =====
  //O micro da placa calcula o angulo a ~175 Hz (pulsos do INT medidos em 03/11), mas a UART dele
  //fica presa em 9600 baud: cada quadro de 13 bytes leva 13,5 ms => no maximo ~50-70 quadros/s.
  //Com PS=GND o MESMO micro responde por I2C no endereco 0x52, nos MESMOS fios (TX/DA = SDA, RX/CL = SCL).
  //A 400 kHz uma leitura de 8 bytes leva ~0,25 ms => da para pegar TODOS os angulos que o micro calcula.
  //GY_MODO: 1 = tenta I2C 0x52 primeiro (mais rapido) e, se nao der, religa o modulo em UART sozinho.
  //         0 = so UART (comportamento antigo).
  #define GY_MODO        1
  #define GY_I2C_ADDR    0x52
  #define GY_I2C_HZ      400000
  #define GY_I2C_PER_US  2500     //intervalo entre leituras I2C (400 Hz de consulta > ~175 Hz do micro)
  uint8_t  gyModoAtivo = 0;       //0 = UART, 1 = I2C
  int8_t   gyI2cOrdem = 0;        //0 = byte alto primeiro (igual UART), 1 = byte baixo primeiro (auto-detectado)
  uint32_t gyI2cUltUs = 0, gyI2cFalhas = 0;
  uint8_t  gyI2cAnt[8] = {0};
  uint8_t  gyEst = 0, gyReg = 0, gyN = 0, gyIdx = 0, gySoma = 0, gyDados[32];
  uint32_t gyQuadros = 0, gyChkErros = 0, gyUltimoUs = 0, gyUltimoMs = 0, gyBytes = 0;
  float    gyRollBruto = 0, gyPitchBruto = 0, gyYawBruto = 0, gyTara = 0, gyTaraPitch = 0, gyRollAnt = 0;
  bool     gyValido = false, gyTemAnt = false;

  void gyTratouQuadro() {
    if (gyReg != 0x14 || gyN != 8) return;
    auto s16 = [&](int i) { return (int16_t)((gyDados[i] << 8) | gyDados[i + 1]); };
    float r = s16(0) / 100.0f, pi = s16(2) / 100.0f, y = s16(4) / 100.0f;
    gyRollBruto = r; gyPitchBruto = pi; gyYawBruto = y;
    uint32_t agora = micros();
    float campo = (GY_CAMPO_ROLL == 0) ? r : pi;
    float outro = (GY_CAMPO_ROLL == 0) ? pi : r;
    float novoRoll = GY_SGN_ROLL * (campo - gyTara);
    if (gyTemAnt && gyUltimoUs) {
      float dtq = (agora - gyUltimoUs) * 1e-6f;
      if (dtq > 0.004f && dtq < 0.25f) {                 //velocidade = d(angulo)/dt entre quadros
        float vel = (novoRoll - gyRollAnt) / dtq;
        GyroX = (1.0f - GY_FILTRO_VEL) * GyroX + GY_FILTRO_VEL * vel;
      }
    }
    roll_IMU = novoRoll; gyRollAnt = novoRoll; gyTemAnt = true;
    pitch_IMU = outro - gyTaraPitch; yaw_IMU = y;
    GyroY = 0; GyroZ = 0;
    gyUltimoUs = agora; gyUltimoMs = millis(); gyValido = true; gyQuadros++;
  }
  void gyAlimenta(uint8_t b) {
    gyBytes++;
    switch (gyEst) {
      case 0: if (b == 0xA4) { gySoma = b; gyEst = 1; } break;
      case 1: if (b == 0x03) { gySoma += b; gyEst = 2; } else gyEst = (b == 0xA4) ? 1 : 0; break;
      case 2: gyReg = b; gySoma += b; gyEst = 3; break;
      case 3: gyN = b; if (gyN > 32) { gyEst = 0; break; } gySoma += b; gyIdx = 0; gyEst = gyN ? 4 : 5; break;
      case 4: gyDados[gyIdx++] = b; gySoma += b; if (gyIdx >= gyN) gyEst = 5; break;
      case 5: if (b == gySoma) gyTratouQuadro(); else gyChkErros++; gyEst = 0; break;
    }
  }
  //Le 8 bytes (roll, pitch, yaw, temp) do registrador 0x14 do micro por I2C. true = leu sem erro.
  bool gyI2cLe8(uint8_t* b) {
    Wire.beginTransmission(GY_I2C_ADDR); Wire.write((uint8_t)0x14);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)GY_I2C_ADDR, (uint8_t)8) != 8) return false;
    for (int i = 0; i < 8; i++) b[i] = Wire.read();
    return true;
  }
  void gyAtualizaI2C() {
    uint32_t agora = micros();
    if (agora - gyI2cUltUs < GY_I2C_PER_US) return;
    gyI2cUltUs = agora;
    uint8_t b[8];
    if (!gyI2cLe8(b)) { gyI2cFalhas++; return; }
    if (memcmp(b, gyI2cAnt, 8) == 0) { gyUltimoMs = millis(); return; }   //micro ainda nao calculou angulo novo (vivo)
    memcpy(gyI2cAnt, b, 8);
    for (int i = 0; i < 8; i += 2) {             //coloca na ordem do quadro UART (byte alto primeiro)
      gyDados[i]     = gyI2cOrdem ? b[i + 1] : b[i];
      gyDados[i + 1] = gyI2cOrdem ? b[i]     : b[i + 1];
    }
    gyReg = 0x14; gyN = 8; gyTratouQuadro();
  }
  void gyAtualiza() {
    if (gyModoAtivo == 1) gyAtualizaI2C();
    else while (Serial2.available()) gyAlimenta((uint8_t)Serial2.read());
  }
  uint32_t gyIdadeMs() { return gyValido ? (millis() - gyUltimoMs) : 99999; }

  void gyLigaModulo(bool psAlto = true) {   //PS=1 -> UART (validado 03/11) | PS=0 -> I2C 0x52 (validado na varredura de 03/11)
    pinMode(GY_PINO_VCC, OUTPUT); digitalWrite(GY_PINO_VCC, LOW);
    const int p[5] = {GY_PINO_SCL, GY_PINO_SDA, GY_PINO_SA0, GY_PINO_CS, GY_PINO_PS};
    for (int i = 0; i < 5; i++) { pinMode(p[i], OUTPUT); digitalWrite(p[i], LOW); }
    delay(300);
    pinMode(GY_PINO_PS, OUTPUT); digitalWrite(GY_PINO_PS, psAlto ? HIGH : LOW);   //PS e lido so quando o modulo liga
    pinMode(GY_PINO_CS, OUTPUT); digitalWrite(GY_PINO_CS, HIGH);
    pinMode(GY_PINO_SA0, OUTPUT); digitalWrite(GY_PINO_SA0, LOW);
    pinMode(GY_PINO_SCL, INPUT); pinMode(GY_PINO_SDA, INPUT);      //so o micro usa o barramento do chip
    gpio_set_drive_capability((gpio_num_t)GY_PINO_VCC, GPIO_DRIVE_CAP_3);
    digitalWrite(GY_PINO_VCC, HIGH);
  }
  //Tenta o modo I2C (PS=GND). Confere se o micro responde em 0x52 e se o angulo faz sentido.
  bool gyTentaI2C() {
    gyLigaModulo(false);
    Wire.begin(GY_PINO_RX, GY_PINO_TX, GY_I2C_HZ);     //SDA = D16 (TX/DA), SCL = D17 (RX/CL)
    Wire.setTimeOut(5);
    Serial.println("GY-601N1 I2C 0x52 (VERSAO TURBO): aguardando o micro da placa (autocalibracao ~3 s, viga PARADA)...");
    delay(3000);
    uint8_t b[8]; int ok = 0;
    for (int t = 0; t < 20; t++) { if (gyI2cLe8(b)) ok++; delay(10); }
    if (ok < 18) { Serial.printf("GY I2C: so %d/20 leituras ok -> volta para UART\n", ok); return false; }
    //auto-detecta a ordem dos bytes: com a viga parada |roll|,|pitch| < 90 graus e temperatura 0..80 C
    for (int ordem = 0; ordem < 2; ordem++) {
      auto v = [&](int i) { return (int16_t)(ordem ? ((b[i + 1] << 8) | b[i]) : ((b[i] << 8) | b[i + 1])) / 100.0f; };
      float r = v(0), p = v(2), te = v(6);
      Serial.printf("GY I2C ordem %d: roll=%.2f pitch=%.2f temp=%.2f\n", ordem, r, p, te);
      if (fabsf(r) < 90 && fabsf(p) < 90 && te > 0 && te < 80) { gyI2cOrdem = ordem; gyModoAtivo = 1; break; }
    }
    if (!gyModoAtivo) { Serial.println("GY I2C: dados sem sentido -> volta para UART"); return false; }
    uint32_t t0 = millis(), q0 = gyQuadros;
    while (millis() - t0 < 2000) { gyAtualiza(); delayMicroseconds(200); }
    float taxa = (gyQuadros - q0) / 2.0f;
    Serial.printf("GY-601N1 I2C 400 kHz: %.0f angulos novos/s (UART 9600 dava ~50) falhas=%lu\n", taxa, (unsigned long)gyI2cFalhas);
    if (taxa < 20) { Serial.println("GY I2C: taxa baixa demais -> volta para UART"); gyModoAtivo = 0; return false; }
    return true;
  }
  void gyInit() {
  #if GY_MODO == 1
    if (gyTentaI2C()) return;
    Wire.end();
    pinMode(GY_PINO_RX, INPUT); pinMode(GY_PINO_TX, INPUT);
  #endif
    gyModoAtivo = 0;
    gyLigaModulo(true);
    Serial2.begin(9600, SERIAL_8N1, GY_PINO_RX, GY_PINO_TX);
    Serial.println("GY-601N1 UART 9600: aguardando o micro da placa (autocalibracao ~3 s, viga PARADA)...");
    uint32_t t0 = millis();
    while (millis() - t0 < 3000) { gyAtualiza(); delay(1); }
    Serial2.write((uint8_t)0xA5); Serial2.write((uint8_t)0x52);    //saida automatica (~50 quadros/s)
    t0 = millis(); uint32_t q0 = gyQuadros;
    while (millis() - t0 < 4000) { gyAtualiza(); delay(1); }
    uint32_t nq = gyQuadros - q0;
    Serial.printf("GY-601N1 UART: quadros_em_4s=%lu chk_erros=%lu bytes=%lu (taxa ~%.0f quadros/s)\n", (unsigned long)nq, (unsigned long)gyChkErros, (unsigned long)gyBytes, nq / 4.0);
    if (nq < 20) {
      Serial.println("GY-601N1 initialization unsuccessful");
      Serial.println("Confira: TX/DA->D16, RX/CL->D17, PS->D27, VCC->D23 (ou 3V3), GND; religue o USB");
      while (1) { delay(1000); Serial.println("GY-601N1 nao respondeu - motores NAO armados"); }
    }
  }
  //Tara: media do roll do micro com a viga parada/nivelada (equivale a calculate_IMU_error)
  void gyFazTara() {
    uint32_t t0 = millis();
    while (millis() - t0 < 1500) { gyAtualiza(); delay(1); }          //descarta o inicio
    double sr = 0, sp = 0; uint32_t n = 0, q0 = gyQuadros; t0 = millis();
    while (n < 150 && millis() - t0 < 10000) {
      gyAtualiza();
      if (gyQuadros != q0) { q0 = gyQuadros; sr += (GY_CAMPO_ROLL == 0 ? gyRollBruto : gyPitchBruto); sp += (GY_CAMPO_ROLL == 0 ? gyPitchBruto : gyRollBruto); n++; }
      delay(1);
    }
    if (n > 0) { gyTara = sr / n; gyTaraPitch = sp / n; }
    gyTemAnt = false; GyroX = 0;
    Serial.printf("GY tara: roll=%.2f pitch=%.2f (n=%lu quadros)\n", gyTara, gyTaraPitch, (unsigned long)n);
  }
#endif

//=====================================================================================
// BMI323 por SPI (GY-601N1, lado do chip)
// Reaproveita o que ja foi validado
// (SPI 8 MHz: 50/50 leituras OK; PS SOLTO = micro da placa quieto, sem corromper o SPI).
// Pinos: SCLK=D18  MOSI=D19  MISO(SA0/SDO)=D26  CS=D25  PS=D27  VCC=D23 (ou 3V3)
// Dado bruto: ACC +-8 g (4096 LSB/g), GYR +-2000 dps (16,384 LSB/dps), ODR 800 Hz.
//=====================================================================================
#if USE_BMI323_SPI
  #include <driver/gpio.h>
  #define BMI_PINO_SCLK 18
  #define BMI_PINO_MOSI 19
  #define BMI_PINO_MISO 26
  #define BMI_PINO_CS   25
  #define BMI_PINO_PS   27
  #define BMI_PINO_VCC  23
  #define BMI_SPI_HZ    8000000
  //Escalas do BMI323 (substituem as do MPU6050 definidas acima)
  #undef  GYRO_SCALE_FACTOR
  #undef  ACCEL_SCALE_FACTOR
  #define GYRO_SCALE_FACTOR  16.384
  #define ACCEL_SCALE_FACTOR 4096.0
  //Mapeamento de eixos/sinais: o MPU6050 e o BMI323 nao estao montados do mesmo jeito.
  //BMI_IDX_x = qual eixo do BMI (0=X,1=Y,2=Z) alimenta o eixo x do firmware; BMI_SGN_x = +1/-1.
  //Ajustar com o teste de repouso (viga com motor 2 para baixo => roll negativo, como no MPU).
  #define BMI_IDX_X 0
  #define BMI_IDX_Y 1
  #define BMI_IDX_Z 2
  #define BMI_SGN_X  1
  #define BMI_SGN_Y  1
  #define BMI_SGN_Z  1
  //Mesmos valores validados no teste do BMI323 por SPI: ACC/GYR 800 Hz, alto desempenho, banda ODR/2
  static const uint16_t BMI_ACC_CFG = 0x702B, BMI_GYR_CFG = 0x704B;
  SPIClass bmiSpi(HSPI);
  bool    bmiOk = false;
  int16_t bmiUlt[6] = {0,0,0,0,0,0};   //ultima amostra valida (ax,ay,az,gx,gy,gz) ja mapeada
  uint32_t bmiLeituras = 0, bmiAmostrasNovas = 0, bmiFalhas = 0;

  void bmiLeRegs(uint8_t reg, uint16_t* dst, uint8_t n) {   //SPI: 1 byte "falso" antes do dado
    bmiSpi.beginTransaction(SPISettings(BMI_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(BMI_PINO_CS, LOW);
    bmiSpi.transfer(0x80 | reg); bmiSpi.transfer(0);
    for (uint8_t i = 0; i < n; i++) { uint8_t lo = bmiSpi.transfer(0), hi = bmiSpi.transfer(0); dst[i] = (uint16_t)(hi << 8 | lo); }
    digitalWrite(BMI_PINO_CS, HIGH); bmiSpi.endTransaction();
  }
  uint16_t bmiLe16(uint8_t reg) { uint16_t v = 0xFFFF; bmiLeRegs(reg, &v, 1); return v; }
  bool bmiEscreveConfere(uint8_t reg, uint16_t v) {
    for (int t = 0; t < 5; t++) {
      bmiSpi.beginTransaction(SPISettings(BMI_SPI_HZ, MSBFIRST, SPI_MODE0));
      digitalWrite(BMI_PINO_CS, LOW); bmiSpi.transfer(reg & 0x7F); bmiSpi.transfer(v & 0xFF); bmiSpi.transfer(v >> 8);
      digitalWrite(BMI_PINO_CS, HIGH); bmiSpi.endTransaction(); delay(2);
      if (bmiLe16(reg) == v) return true;
    }
    return false;
  }
  void bmiLigaModulo() {   //PS SOLTO = micro da placa quieto; CS alto na partida
    pinMode(BMI_PINO_VCC, OUTPUT); digitalWrite(BMI_PINO_VCC, LOW);
    const int p[4] = {BMI_PINO_SCLK, BMI_PINO_MOSI, BMI_PINO_MISO, BMI_PINO_CS};
    for (int i = 0; i < 4; i++) { pinMode(p[i], OUTPUT); digitalWrite(p[i], LOW); }
    pinMode(BMI_PINO_PS, OUTPUT); digitalWrite(BMI_PINO_PS, LOW); delay(300);
    pinMode(BMI_PINO_PS, INPUT);                                   //PS solto
    pinMode(BMI_PINO_CS, OUTPUT); digitalWrite(BMI_PINO_CS, HIGH);
    pinMode(BMI_PINO_MISO, OUTPUT); digitalWrite(BMI_PINO_MISO, LOW);
    pinMode(BMI_PINO_MOSI, INPUT_PULLUP); pinMode(BMI_PINO_SCLK, INPUT_PULLUP);
    gpio_set_drive_capability((gpio_num_t)BMI_PINO_VCC, GPIO_DRIVE_CAP_3);
    digitalWrite(BMI_PINO_VCC, HIGH); delay(300);
  }
  void bmiInit() {
    bmiLigaModulo();
    bmiSpi.begin(BMI_PINO_SCLK, BMI_PINO_MISO, BMI_PINO_MOSI, BMI_PINO_CS);   //SCLK, MISO, MOSI, CS
    bmiLe16(0); bmiLe16(0);                                         //1a leitura troca o chip para SPI
    bool id = false;
    for (int t = 0; t < 20 && !id; t++) { id = ((bmiLe16(0) & 0xFF) == 0x43); delay(20); }
    bool cfg = id && bmiEscreveConfere(0x20, BMI_ACC_CFG) && bmiEscreveConfere(0x21, BMI_GYR_CFG);
    bmiOk = id && cfg;
    Serial.printf("BMI323 SPI %d MHz: chip_id_ok=%d config_ok=%d (ACC 0x%04X GYR 0x%04X)\n", BMI_SPI_HZ/1000000, id, cfg, BMI_ACC_CFG, BMI_GYR_CFG);
    if (!bmiOk) {
      Serial.println("BMI323 initialization unsuccessful");
      Serial.println("Confira: SCL->D18, SDA->D19, SA0->D26, CS->D25, PS->D27 (ou solto), VCC; religue o USB");
      while (1) { delay(1000); Serial.println("BMI323 nao respondeu - motores NAO armados"); }
    }
    delay(50);
  }
  //Le o BMI323. Se ainda nao ha amostra nova (loop a 2 kHz, ODR 800 Hz) devolve a ultima valida.
  void bmiLeitura6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint16_t r[7];
    bmiLeRegs(0x02, r, 7);                       //0x02 status, 0x03..0x05 acc, 0x06..0x08 gyr
    bmiLeituras++;
    if ((r[0] & 0xC0) == 0xC0) {                 //acc e gyr com dado novo
      int16_t a[3] = {(int16_t)r[1], (int16_t)r[2], (int16_t)r[3]};
      int16_t g[3] = {(int16_t)r[4], (int16_t)r[5], (int16_t)r[6]};
      bmiUlt[0] = BMI_SGN_X * a[BMI_IDX_X]; bmiUlt[1] = BMI_SGN_Y * a[BMI_IDX_Y]; bmiUlt[2] = BMI_SGN_Z * a[BMI_IDX_Z];
      bmiUlt[3] = BMI_SGN_X * g[BMI_IDX_X]; bmiUlt[4] = BMI_SGN_Y * g[BMI_IDX_Y]; bmiUlt[5] = BMI_SGN_Z * g[BMI_IDX_Z];
      bmiAmostrasNovas++;
    }
    *ax = bmiUlt[0]; *ay = bmiUlt[1]; *az = bmiUlt[2]; *gx = bmiUlt[3]; *gy = bmiUlt[4]; *gz = bmiUlt[5];
  }
#endif
//Leitura unica usada por getIMUdata() e calculate_IMU_error()
void lerIMU6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  #if USE_BMI323_SPI
    bmiLeitura6(ax, ay, az, gx, gy, gz);
  #else
    mpu6050.getMotion6(ax, ay, az, gx, gy, gz);
  #endif
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works.
   */
  #if USE_GY_UART
    gyInit();
    return;
  #endif
  #if USE_BMI323_SPI
    bmiInit();
    return;
  #endif
  #if defined USE_MPU6050_I2C
    Wire.begin();
    //I2C a 400 kHz (maximo do datasheet do MPU6050): a fiacao em protoboard/jumper nao suporta 1 MHz.
    //Se houver problema de timing/I2C, e possivel ajustar esse valor.
    Wire.setClock(400000);
    
    mpu6050.initialize();
    
    //A 1a leitura logo apos ligar/reset costuma falhar (o sensor/barramento esta subindo).
//Em vez de abortar na 1a tentativa, tenta ate 20x (50 ms entre elas = ~1 s).
//Se todas falharem, imprime diagnostico de barramento antes de travar.
    bool conn = false;
    for (int t = 0; t < 20 && !conn; t++) {
      conn = (mpu6050.testConnection() == true);
      if (!conn) {
        mpu6050.initialize();          // reacorda/reconfigura e tenta de novo
        delay(50);
      }
    }

    if (!conn) {
      //--- DIAGNOSTICO DE BARRAMENTO ---
      int sdaLvl = digitalRead(21);    //niveis dos fios (repouso I2C = 1/1)
      int sclLvl = digitalRead(22);
      Wire.beginTransmission(0x68);    //prova de endereco: 0 = chip ACK
      int err = Wire.endTransmission();
      uint8_t who = 0;
      Wire.beginTransmission(0x68);
      Wire.write(0x75);                //WHO_AM_I cru
      uint8_t e2 = Wire.endTransmission(false);
      uint8_t n = Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
      if (n == 1 && Wire.available()) who = Wire.read();
      Serial.print("DIAG I2C: ack="); Serial.print(err == 0 ? "SIM" : "NAO");
      Serial.print(" errTrans="); Serial.print(err);
      Serial.print(" sda21="); Serial.print(sdaLvl);
      Serial.print(" scl22="); Serial.print(sclLvl);
      Serial.print(" who=0x"); Serial.print(who, HEX);
      Serial.print(" regErr="); Serial.print(e2);
      Serial.print(" nLidos="); Serial.println(n);

      if (err == 0 && who != 0) {
        //o chip RESPONDE mas o ID nao bateu com 0x34 (clona?) - segue com
        //leitura direta do registrador
        Serial.println("AVISO: chip respondeu mas testConnection()!=0x34 - seguindo mesmo assim");
      } else {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(1) {}
      }
    } else {
      Serial.println("MPU6050 OK (testConnection apos retry)");
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
  #elif defined USE_MPU9250_SPI
    int status = mpu9250.begin();    

    if (status < 0) {
      Serial.println("MPU9250 initialization unsuccessful");
      Serial.println("Check MPU9250 wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
  #endif
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  #if USE_GY_UART
    gyAtualiza();   //angulo e velocidade ja atualizados nos quadros recebidos
    return;
  #endif

  #if defined USE_MPU6050_I2C
    lerIMU6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //===== medicao de vibracao, IMEDIATAMENTE apos escala/filtro da IMU =====
  //Calculada com as mesmas aceleracoes em g que alimentam o Madgwick (ja escaladas e
  //com o LP filtro B_accel aplicado). dev = | |a| - 1g |; vibRms = EWMA (alpha=0.01);
  //vibPeak = maximo da janela de 1 s (zerado em printStatsIfDue()).
  {
    float dev = fabs(sqrtf(AccX*AccX + AccY*AccY + AccZ*AccZ) - 1.0f);
    vibRms += (dev - vibRms)*0.01f;
    if (dev > vibPeak) vibPeak = dev;
  }

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  #if USE_GY_UART
    gyFazTara();
    return;
  #endif
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  #if USE_BMI323_SPI
    const int N_CALIB = 4000;   //BMI323: so conta amostra NOVA (ODR 800 Hz) => ~5 s, igual ao MPU
  #else
    const int N_CALIB = 12000;
  #endif
  while (c < N_CALIB) {
    #if USE_BMI323_SPI
      { uint32_t antes = bmiAmostrasNovas; lerIMU6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ); if (bmiAmostrasNovas == antes) continue; }
    #elif defined USE_MPU6050_I2C
      lerIMU6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  #if USE_GY_UART
    return;   //GY_UART: roll/pitch vem prontos do micro (sem fusao no ESP32)
  #endif
  //use 6DOF algorithm if MPU6050 is being used
  #if defined USE_MPU6050_I2C 
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  #endif
  
  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */
  //PWM de servo 1000-2000us para os ESCs SimonK HW-30A (400 Hz).
  //(Original OneShot125: scaled*125 + 125, limitado a 125..250us)
  m1_command_PWM = m1_command_scaled*1000 + 1000;
  m2_command_PWM = m2_command_scaled*1000 + 1000;
  m3_command_PWM = m3_command_scaled*1000 + 1000;
  m4_command_PWM = m4_command_scaled*1000 + 1000;
  m5_command_PWM = m5_command_scaled*1000 + 1000;
  m6_command_PWM = m6_command_scaled*1000 + 1000;
  //Constrain commands to motors within standard PWM ESC bounds
  m1_command_PWM = constrain(m1_command_PWM, 1000, 2000);
  m2_command_PWM = constrain(m2_command_PWM, 1000, 2000);
  m3_command_PWM = constrain(m3_command_PWM, 1000, 2000);
  m4_command_PWM = constrain(m4_command_PWM, 1000, 2000);
  m5_command_PWM = constrain(m5_command_PWM, 1000, 2000);
  m6_command_PWM = constrain(m6_command_PWM, 1000, 2000);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
    
  #elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;  
      float bias  = 895.0; 
      channel_1_pwm = sbusChannels[0] * scale + bias;
      channel_2_pwm = sbusChannels[1] * scale + bias;
      channel_3_pwm = sbusChannels[2] * scale + bias;
      channel_4_pwm = sbusChannels[3] * scale + bias;
      channel_5_pwm = sbusChannels[4] * scale + bias;
      channel_6_pwm = sbusChannels[5] * scale + bias; 
    }

  #elif defined USE_DSM_RX
    if (DSM.timedOut(micros())) {
        //Serial.println("*** DSM RX TIMED OUT ***");
    }
    else if (DSM.gotNewFrame()) {
        uint16_t values[num_DSM_channels];
        DSM.getChannelValues(values, num_DSM_channels);

        channel_1_pwm = values[0];
        channel_2_pwm = values[1];
        channel_3_pwm = values[2];
        channel_4_pwm = values[3];
        channel_5_pwm = values[4];
        channel_6_pwm = values[5];
    }
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void commandMotors() {
  //DESCRIPTION: Send PWM pulses to motor pins (standard servo PWM)
  /*
   * Os ESCs SimonK HW-30A nao interpretam OneShot125. Usa ESP32Servo nos pinos m1-m4
   * (400 Hz, 1000-2000 us). Os valores mX_command_PWM vao em microssegundos, calculados em scaleCommands().
   */
  esc1.writeMicroseconds(m1_command_PWM);
  esc2.writeMicroseconds(m2_command_PWM);
  //esc3/esc4 removidos (versao 2 motores)
}

void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs() {
   //2 motores: comeca no MAXIMO para o procedimento classico de calibracao
   modoCalibraESC = true;
   serThrottle = 2000;
   Serial.println("=== MODO CALIBRACAO ESC (SEM HELICE!) ===");
   Serial.println("Throttle em 2000. 1) Ligue a BATERIA dos ESCs agora. 2) Apos os bipes, digite 0 (minimo).");
   Serial.println("3) Apos os bipes de confirmacao, desligue a bateria. Volte CALIBRAR_ESC para 0 e grave de novo.");
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(ledPin, HIGH); //LED on to indicate we are not in main loop

      //2 motores: sem radio -> throttle vem do Monitor Serial ('9'=maximo, '0'=minimo, '+'/'-')
      pollSerialCommands();
      channel_1_pwm = serThrottle;
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      
      m1_command_scaled = thro_des;
      m2_command_scaled = thro_des;
      m3_command_scaled = 0; //2 motores
      m4_command_scaled = 0;
      m5_command_scaled = thro_des;
      m6_command_scaled = thro_des;
      s1_command_scaled = thro_des;
      s2_command_scaled = thro_des;
      s3_command_scaled = thro_des;
      s4_command_scaled = thro_des;
      s5_command_scaled = thro_des;
      s6_command_scaled = thro_des;
      s7_command_scaled = thro_des;
      scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
    
      //throttleCut(); //Directly sets motor commands to low based on state of ch5
      
      servo1.write(s1_command_PWM); 
      servo2.write(s2_command_PWM);
      servo3.write(s3_command_PWM);
      servo4.write(s4_command_PWM);
      servo5.write(s5_command_PWM);
      servo6.write(s6_command_PWM);
      servo7.write(s7_command_PWM);
      commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
      
      //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *  
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //Constrain param within max bounds
  
  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*  
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. 
   *  
   */
  if (param > param_des) { //Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { //Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); //Constrain param within max bounds
  
  return param;
}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not 
   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis. 
   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the 
   * IMU tilted 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw*roll_des;
  roll_des = reverseRoll*switch_holder;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (1000 is
      minimum for the standard PWM servo ESC output used here - no original era 120, minimo do OneShot125) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 1000us only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if ((channel_5_pwm > 1500) || (armedFly == false)) {
    armedFly = false;
    m1_command_PWM = 1000;
    m2_command_PWM = 1000;
    m3_command_PWM = 1000;
    m4_command_PWM = 1000;
    m5_command_PWM = 1000;
    m6_command_PWM = 1000;

    //Uncomment if using servo PWM variables to control motor ESCs
    //s1_command_PWM = 0;
    //s2_command_PWM = 0;
    //s3_command_PWM = 0;
    //s4_command_PWM = 0;
    //s5_command_PWM = 0;
    //s6_command_PWM = 0;
    //s7_command_PWM = 0;
  }
}

void calibrateMagnetometer() {
  #if defined USE_MPU9250_SPI 
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
      Serial.print("float MagErrorX = ");
      Serial.print(mpu9250.getMagBiasX_uT());
      Serial.println(";");
      Serial.print("float MagErrorY = ");
      Serial.print(mpu9250.getMagBiasY_uT());
      Serial.println(";");
      Serial.print("float MagErrorZ = ");
      Serial.print(mpu9250.getMagBiasZ_uT());
      Serial.println(";");
      Serial.print("float MagScaleX = ");
      Serial.print(mpu9250.getMagScaleFactorX());
      Serial.println(";");
      Serial.print("float MagScaleY = ");
      Serial.print(mpu9250.getMagScaleFactorY());
      Serial.println(";");
      Serial.print("float MagScaleZ = ");
      Serial.print(mpu9250.getMagScaleFactorZ());
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }
  
    while(1); //Halt code so it won't enter main loop until this function commented out
  #endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); //Halt code so it won't enter main loop until this function commented out
}

//========================================================================================================================//
//                              COMANDO SERIAL + ESTATISTICAS                                          //
//========================================================================================================================//

void pollSerialCommands() {
  //DESCRIPTION: Le o comando serial que substitui o receptor de radio desativado.
  /*
   * Teclas (qualquer letra maiuscula ou minúscula):
   *   'a'/'A' -> serCut = false  (libera o corte; armedStatus() pode armar se throttle < 1050)
   *   'd'/'D' -> serCut = true   (corte de acelerador: throttleCut() zera motores e desarma)
   *   '+'     -> serThrottle += 25 us (limitado a 2000)
   *   '-'     -> serThrottle -= 25 us (limitado a 1000)
   *   'p'/'P' -> imprime o estado atual
   * O polling e feito a cada ciclo; o processamento e barato (so Serial.available quando vazio).
   */
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'a': case 'A':
        serCut = false; //libera corte -> proximo ciclo ch5=1000, armedStatus() arma
        Serial.println("CMD: corte LIBERADO (ch5=1000)");
        break;
      case 'd': case 'D':
        serCut = true;  //corta -> proximo ciclo ch5=2000, throttleCut() desarma
        Serial.println("CMD: corte ATIVADO (ch5=2000, desarmado)");
        break;
      case '+':
        serThrottle += 25;
        if (serThrottle > limiteThr) serThrottle = limiteThr; //limite de seguranca (#L)
        Serial.printf("CMD: throttle=%lu\n", serThrottle);
        break;
      case '-':
        serThrottle -= 25;
        if (serThrottle < 1000) serThrottle = 1000; //clamp (unsigned: testa apos decremento)
        Serial.printf("CMD: throttle=%lu\n", serThrottle);
        break;
      case '9':
        if (!modoCalibraESC) { Serial.println("CMD: '9' so funciona no modo CALIBRAR_ESC"); break; }
        serThrottle = 2000;
        Serial.println("CMD: throttle=2000 (MAXIMO)");
        break;
      case '0':
        serThrottle = 1000;
        Serial.println("CMD: throttle=1000 (MINIMO)");
        break;
      case 'p': case 'P':
        Serial.printf("STATE: thr=%lu cut=%d armed=%d ch5=%lu roll=%.1f pitch=%.1f yaw=%.1f vibRms=%.3f\n",
                      serThrottle, serCut ? 1 : 0, armedFly ? 1 : 0, channel_5_pwm,
                      roll_IMU, pitch_IMU, yaw_IMU, (double)vibRms);
        break;
      case 'c': case 'C':
        //Recalibracao manual (drone PARADO e nivelado). Bloqueia o loop ~5 s, entao so
        //e permitida desarmado/throttle minimo - com motores rodando seria perigoso.
        if (armedFly || channel_1_pwm > 1060) {
          Serial.println("CMD: recalibra REPROVADA (desarme e volte throttle p/ 1000 primeiro)");
        } else {
          Serial.println("CMD: recalibrando (~5 s, drone PARADO e nivelado)...");
          calculate_IMU_error();
          Serial.println("CMD: recalibracao concluida");
        }
        break;
      case '#': { //comando de linha: #P0.31  #I0.4  #D0.14  #L1400 (limite thr)  #T1200 (thr direto)  #A45
        char buf[24]; int n = 0; unsigned long t = millis();
        while (millis() - t < 30 && n < 23) {
          if (Serial.available()) { char ch = Serial.read(); if (ch == '\n' || ch == '\r') break; buf[n++] = ch; }
        }
        buf[n] = 0;
        if (n < 2) break;
        float v = atof(buf + 1);
        switch (buf[0]) {
          case 'P': Kp_roll_angle = Kp_pitch_angle = v; break;
          case 'I': Ki_roll_angle = Ki_pitch_angle = v; break;
          case 'D': Kd_roll_angle = Kd_pitch_angle = v; break;
          case 'L': limiteThr = constrain((unsigned long)v, 1000UL, 2000UL);
                    if (serThrottle > limiteThr) serThrottle = limiteThr; break;
          case 'T': serThrottle = constrain((unsigned long)v, 1000UL, limiteThr); break;
          case 'A': anguloMaxDesarme = constrain(v, 5.0f, 80.0f); break;
          default: Serial.println("CMD: # desconhecido"); break;
        }
      } //cai no 'g' para confirmar os valores
      case 'g': case 'G':
        Serial.printf("GANHOS: Kp=%.4f Ki=%.4f Kd=%.4f limiteThr=%lu thr=%lu angMax=%.0f\n",
                      Kp_roll_angle, Ki_roll_angle, Kd_roll_angle, limiteThr, serThrottle, anguloMaxDesarme);
        break;
      case 'l': case 'L':
        logFast = !logFast;
        logFastN = 0;
        Serial.printf("CMD: log rapido %s (100Hz: LOG,ms,roll,pitch,gx,gy,gz,pidR,pidP,pidY,m1,m2,m3,m4)\n",
                      logFast ? "LIGADO" : "DESligado");
        break;
      default:
        break; //ignora CR/LF e qualquer outra tecla
    }
  }
}

void printStatsIfDue() {
  //DESCRIPTION: Imprime a linha STATS a cada 1 s e zera a janela de acumulacao.
  /*
   * Formato:
   *   STATS,<bodyAvgUs>,<bodyMaxUs>,<estouros>,<perAvgUs>,<perMaxUs>,<perMinUs>,<vibRmsG>,<vibPeakG>,<alvoUs>,<ciclos>
   * O UART tem buffer TX (256 bytes) no ESP32, entao a linha (~90 bytes) e copiada para o
   * buffer e transmitida em background - o atraso no loop e de poucos us, nao ~2 ms.
   */
  if (millis() - statsPrintTimerMs < 250) return; //DEMO: 4 linhas por segundo
  statsPrintTimerMs = millis();
  Serial.printf("roll: %6.1f   pitch: %6.1f\n", (double)roll_IMU, (double)pitch_IMU); //DEMO: so o angulo
  //Reset da janela de 1 s (inclusive vibPeak; vibRms tambem zera para a janela seguinte)
  statsBodySum = 0; statsBodyMax = 0; statsEstouros = 0; statsCiclos = 0;
  statsPerSum = 0; statsPerMax = 0; statsPerMin = 0xFFFFFFFF;
  vibPeak = 0.0f;
  vibRms = 0.0f;
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(ledPin, blinkAlternate); //ledPin (GPIO2)
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(ledPin, LOW);
    delay(downTime);
    digitalWrite(ledPin, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6:"));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(MagX);
    Serial.print(F(" MagY:"));
    Serial.print(MagY);
    Serial.print(F(" MagZ:"));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command:"));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command:"));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command:"));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("s1_command:"));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command:"));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command:"));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command:"));
    Serial.print(s4_command_PWM);
    Serial.print(F(" s5_command:"));
    Serial.print(s5_command_PWM);
    Serial.print(F(" s6_command:"));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command:"));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}

//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //raiz quadrada inversa
}
