# Código MPU6050 (I2C)

✅ **Pronto para rodar.** Estabiliza a haste com o sensor **MPU6050** lido por **I2C**.

## Tecnologia

- **ESP32** rodando o laço do dRehmFlight a **2 kHz**
- **MPU6050** (giroscópio + acelerômetro de 6 eixos) no barramento I2C a **400 kHz**, endereço `0x68`. É o máximo do datasheet; a fiação de protoboard não aguenta 1 MHz.
- Faixas: giroscópio **±250 °/s** e acelerômetro **±2 g**
- O sensor entrega dados novos a ~1 kHz. Eles passam pelo passa-baixa e pelo **filtro de Madgwick** no próprio ESP32, que calcula roll e pitch.
- PID de ângulo (Kp 0,2 · Ki 0,3 · Kd 0,05) gera o comando dos 2 ESCs a **400 Hz / 1000–2000 µs**

## Ligações

| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

| ESC | ESP32 |
|---|---|
| Sinal do ESC 1 (motor esquerdo) | GPIO 32 |
| Sinal do ESC 2 (motor direito) | GPIO 33 |
| GND dos ESCs | GND do ESP32 (terra comum) |

## Como rodar

Grave o `Codigo_MPU6050_I2C.ino` (com `radioComm.ino` e `src/` na mesma pasta), abra o monitor serial a **500000 baud** e deixe a haste parada durante a calibração. O acelerador sobe sozinho até 1150 µs. Os comandos e as precauções de segurança estão no [README da pasta principal](../README.md).
