# Código BMI323 (I2C)

**Pronto para rodar.** Estabiliza a haste com o sensor **BMI323** (placa GY-601N1) lido direto por **I2C**, o mesmo protocolo do MPU6050. Serve para comparar o sensor (BMI323 × MPU6050) e o protocolo (I2C × SPI) separadamente.

## Tecnologia

- **ESP32** rodando o laço do dRehmFlight a **2 kHz**
- **BMI323** (IMU Bosch de 6 eixos) por **I2C a 400 kHz**, endereço **0x68**. Se a fiação não aguentar no boot, o código cai sozinho para **100 kHz** (e avisa que o laço fica mais lento).
- Barramento `TwoWire(1)` (o segundo controlador I2C do ESP32)
- No I2C do BMI323, depois do endereço do registrador vêm **2 bytes falsos** antes do dado; o código descarta esses bytes
- Dados brutos: acelerômetro **±8 g** (4096 LSB/g), giroscópio **±2000 °/s** (16,384 LSB/°/s), **ODR de 800 Hz** (registradores `0x20 = 0x702B`, `0x21 = 0x704B`)
- Filtro de **Madgwick** no ESP32 → PID de ângulo (Kp 0,31 · Ki 0,4 · Kd 0,14) → 2 ESCs a **400 Hz / 1000–2000 µs**
- Na partida, mede e mostra no serial quantas amostras novas chegam por segundo

## Ligações

| GY-601N1 / BMI323 | ESP32 |
|---|---|
| SCL | GPIO 18 |
| SDA | GPIO 19 |
| SA0 (SDO) | GPIO 26 (em nível baixo, o endereço fica 0x68) |
| CS | GPIO 25 (em nível **alto** na partida, o que coloca o chip em I2C) |
| PS | solto |
| VCC | GPIO 23 (o ESP32 liga o módulo) ou 3V3 |
| GND | GND |

| ESC | ESP32 |
|---|---|
| ESC 1 (motor esquerdo) | GPIO 32 |
| ESC 2 (motor direito) | GPIO 33 |

## Como rodar

Grave o `Codigo_BMI323_I2C.ino` e abra o serial a **500000 baud**. Ele deve mostrar `BMI323 I2C 400 kHz (0x68): ok=1`; se não mostrar, confira as ligações acima e religue o USB. Deixe a haste parada durante a calibração e o acelerador sobe sozinho. Os comandos estão no [README da pasta principal](../README.md).
