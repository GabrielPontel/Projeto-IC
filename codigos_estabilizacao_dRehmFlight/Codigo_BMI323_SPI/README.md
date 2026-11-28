# Código BMI323 (SPI)

**Pronto para rodar.** Estabiliza a haste com o sensor **BMI323** (placa GY-601N1) lido direto por **SPI**.

## Tecnologia

- **ESP32** rodando o laço do dRehmFlight a **2 kHz**
- **BMI323** (IMU Bosch de 6 eixos) por **SPI modo 0 a 10 MHz**. Se a fiação não aguentar no boot, o código cai sozinho para **8 MHz**, valor já validado.
- Dados brutos: acelerômetro **±8 g** (4096 LSB/g), giroscópio **±2000 °/s** (16,384 LSB/°/s)
- **ODR de 800 Hz** em modo de alto desempenho, banda ODR/2 (registradores `0x20 = 0x702B`, `0x21 = 0x704B`)
  - Existe uma opção "turbo" de **1600 Hz** (`#define BMI_ODR_HZ 1600`). Ela não melhorou a estabilização nos testes (giroscópio mais ruidoso), por isso o padrão é 800 Hz.
- Quando o laço de 2 kHz não encontra amostra nova, reaproveita a última válida
- Filtro de **Madgwick** no ESP32 → PID de ângulo (Kp 0,31 · Ki 0,4 · Kd 0,14) → 2 ESCs a **400 Hz / 1000–2000 µs**
- Na partida, o código mede e mostra no serial quantas amostras novas chegam por segundo

## Ligações

| GY-601N1 / BMI323 | ESP32 |
|---|---|
| SCL / SCLK | GPIO 18 |
| SDA / MOSI | GPIO 19 |
| SA0 / MISO (SDO) | GPIO 26 |
| CS | GPIO 25 |
| PS | GPIO 27 (ou solto: deixa o micro da placa quieto) |
| VCC | GPIO 23 (o ESP32 liga o módulo) ou 3V3 |
| GND | GND |

| ESC | ESP32 |
|---|---|
| ESC 1 (motor esquerdo) | GPIO 32 |
| ESC 2 (motor direito) | GPIO 33 |

## Como rodar

Grave o `Codigo_BMI323_SPI.ino` e abra o serial a **500000 baud**. Ele deve mostrar `chip_id_ok=1 config_ok=1`; se não mostrar, confira as ligações acima. Deixe a haste parada durante a calibração e o acelerador sobe sozinho. Os comandos estão no [README da pasta principal](../README.md).
