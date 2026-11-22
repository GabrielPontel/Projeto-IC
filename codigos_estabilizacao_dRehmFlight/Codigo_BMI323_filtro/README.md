# Código BMI323 com filtro interno (GY-601N1)

✅ **Pronto para rodar.** Usa o **ângulo já calculado e filtrado pelo microcontrolador da placa GY-601N1**, que lê o BMI323 internamente. O ESP32 não roda o Madgwick neste modo.

## Tecnologia

- **ESP32** rodando o laço do dRehmFlight a **2 kHz**
- A placa **GY-601N1** tem o BMI323 e um microcontrolador que calcula roll/pitch/yaw a **~175 Hz** (medido pelos pulsos do pino INT)
- **Modo padrão (`GY_MODO 1`): I2C a 400 kHz no endereço `0x52`**, com o pino PS em GND. O ESP32 consulta a cada **2,5 ms (400 Hz)**, mais rápido que os ~175 Hz do micro, então pega todos os ângulos calculados. Cada leitura de 8 bytes leva ~0,25 ms.
- **Reserva: UART 9600 8N1** (PS em 3V3). Se o I2C falhar, o código religa o módulo em UART sozinho. Os quadros têm 13 bytes `A4 03 14 08 | roll pitch yaw temp | CHK`, em int16/100 com checksum, e chegam a ~50 quadros/s.
- O micro da placa se autocalibra ~3 s depois de ligar (haste **parada**). Na partida o código guarda a "tara" do ângulo.
- Como não há giroscópio bruto neste modo, a **velocidade angular do termo D é derivada do próprio ângulo**, com passa-baixa (`GY_FILTRO_VEL = 0.4`)
- PID de ângulo (Kp 0,2 · Ki 0,3 · Kd 0,05) → 2 ESCs a **400 Hz / 1000–2000 µs**
- O código também tem o caminho por SPI direto ao BMI323 (`USE_BMI323_SPI`), desligado nesta versão

## Ligações

| GY-601N1 | ESP32 |
|---|---|
| TX / DA | GPIO 16 (SDA no I2C · RX2 na UART) |
| RX / CL | GPIO 17 (SCL no I2C · TX2 na UART) |
| PS | GPIO 27 (o código controla: GND = I2C, 3V3 = UART) |
| VCC | GPIO 23 |
| CS | GPIO 25 |
| SA0 | GPIO 26 |
| SCL / SDA do BMI323 | GPIO 18 / 19 (em alta impedância: só o micro da placa conversa com o BMI323) |

| ESC | ESP32 |
|---|---|
| ESC 1 (motor esquerdo) | GPIO 32 |
| ESC 2 (motor direito) | GPIO 33 |

## Como rodar

Grave o `Codigo_BMI323_filtro.ino`, abra o serial a **500000 baud** e deixe a haste **parada** nos primeiros segundos, enquanto o micro da placa se calibra. O serial mostra quantos ângulos novos chegam por segundo pelo I2C. Depois disso o acelerador sobe sozinho. Os comandos estão no [README da pasta principal](../README.md).
