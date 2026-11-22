# 🚁 Códigos de Estabilização a partir do dRehmFlight

Firmwares **prontos para rodar** que estabilizam a bancada da IC — uma haste (pêndulo/balança) com **2 motores brushless** — usando um **ESP32** e controle **PID**. Os três códigos são adaptações do controlador de voo open-source [dRehmFlight](https://github.com/nickrehm/dRehmFlight) (Nicholas Rehm, GPL-3.0), portado do Teensy para o ESP32 e reduzido para 2 motores.

> ✅ **Status: pronto para rodar.** São os códigos usados na apresentação da IC. Cada um compila e roda como está: grave no ESP32, deixe a haste parada e nivelada, e ele calibra o sensor e sobe o acelerador sozinho.

## Os três códigos

| Pasta | Sensor | Comunicação | Taxa do sensor | Como obtém o ângulo |
|---|---|---|---|---|
| [`Codigo_MPU6050_I2C`](Codigo_MPU6050_I2C) | MPU6050 | I2C 400 kHz | ~1 kHz | Giroscópio + acelerômetro → filtro de **Madgwick** no ESP32 |
| [`Codigo_BMI323_SPI`](Codigo_BMI323_SPI) | BMI323 (placa GY-601N1) | SPI 10 MHz (cai para 8 MHz se falhar) | 800 Hz (opção 1600 Hz) | Giroscópio + acelerômetro → filtro de **Madgwick** no ESP32 |
| [`Codigo_BMI323_filtro`](Codigo_BMI323_filtro) | BMI323 + microcontrolador da GY-601N1 | I2C 400 kHz no endereço 0x52 (cai para UART 9600 se falhar) | ~175 ângulos/s | Ângulo **já filtrado pelo micro da placa** |

## O que os três têm em comum

| Item | Valor |
|---|---|
| Microcontrolador | ESP32 DevKit (LED no GPIO 2) |
| Laço de controle | **2 kHz** (`loopRate(2000)`, 500 µs por ciclo) |
| Estimativa de atitude | Filtro de Madgwick (`B_madgwick = 0.04`) + passa-baixa no acelerômetro (`B_accel = 0.14`) e no giroscópio (`B_gyro = 0.1`) |
| Controle | PID de ângulo — **Kp 0,2 · Ki 0,3 · Kd 0,05**, integrador limitado em ±25 |
| Motores | 2 brushless com **ESC SimonK HW-30A** |
| Sinal dos ESCs | PWM de servo a **400 Hz**, pulso de **1000–2000 µs** (biblioteca ESP32Servo) |
| Pinos dos ESCs | **GPIO 32** = motor esquerdo · **GPIO 33** = motor direito |
| Serial | **500000 baud** |
| Acelerador de demonstração | Sobe sozinho até **1150 µs** (25 µs a cada 150 ms), 5 s depois da calibração |
| Segurança | Acelerador limitado a 1400 µs; desarma sozinho se a inclinação passar de **45°** |

O receptor de rádio do dRehmFlight está **desativado**: os pinos padrão dele (21/22) são os mesmos do I2C. Os comandos chegam pelo monitor serial.

## Como rodar

1. Instale a **Arduino IDE** com o pacote de placas **ESP32 (Espressif)**.
2. Copie a pasta [`libraries/ESP32Servo`](libraries/ESP32Servo) para `Documentos/Arduino/libraries/` (ou instale a "ESP32Servo" pelo gerenciador de bibliotecas).
3. Abra o `.ino` da pasta do código escolhido. O `radioComm.ino` e a pasta `src/` precisam ficar juntos dele.
4. Placa **ESP32 Dev Module**, escolha a porta e grave.
5. Abra o monitor serial em **500000 baud**.
6. Deixe a haste **parada e nivelada** durante a calibração (~5 s). Depois disso o acelerador sobe sozinho.

> ⚠️ **Segurança:** teste sem hélices na primeira vez e deixe a bancada presa. Para cortar os motores a qualquer momento, envie `d` no monitor serial.

## Comandos pelo monitor serial

| Comando | O que faz |
|---|---|
| `a` | Libera o corte (permite armar) |
| `d` | **Corta os motores** e desarma |
| `+` / `-` | Acelerador ±25 µs |
| `0` | Acelerador no mínimo (1000 µs) |
| `p` | Mostra o estado (acelerador, armado, roll, pitch, yaw, vibração) |
| `c` | Recalibra o sensor (só com o sistema desarmado) |
| `g` | Mostra os ganhos atuais |
| `l` | Liga/desliga o log rápido de ~100 Hz para gráficos |
| `#P0.2` `#I0.3` `#D0.05` | Ajusta Kp, Ki e Kd ao vivo |
| `#L1400` | Muda o limite do acelerador |
| `#T1200` | Define o acelerador direto (µs) |
| `#A45` | Muda o ângulo de desarme automático |

## Créditos

- Base: [dRehmFlight](https://github.com/nickrehm/dRehmFlight), de Nicholas Rehm (GPL-3.0)
- Filtro de Madgwick adaptado de [arduino-libraries/MadgwickAHRS](https://github.com/arduino-libraries/MadgwickAHRS)
- Biblioteca [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)
- Autor: **Gabriel Pontel de Mori** · Orientador: **Prof. Dr. Ricardo C. C. Ferrari** · IFSP Presidente Epitácio (PIBIFSP)
