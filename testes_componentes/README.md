# Testes de componentes com o ESP32

> Se for usar qualquer um destes componentes (MPU6050, BMI323 / GY-601N1, ESCs e motores brushless), comece por aqui: cada pasta é um teste isolado, com código, README e diagrama de ligação.

Esta pasta reúne **testes simples e independentes** para conferir, um por um, se cada componente da bancada (balança com 2 motores BLDC, ESP32, PID em cascata) está **ligado certo e respondendo**: o protocolo de comunicação dos sensores e os ESCs/motores. Nenhum deles faz controle de voo; servem para **validar fios, protocolo e sinal** antes de usar o componente no firmware principal da balança (que fica fora deste repositório).

## Os testes

| Pasta | Componente | Protocolo / sinal | Pergunta que responde |
|---|---|---|---|
| `teste_1_mpu6050_i2c` | MPU6050 (GY-521) | I2C — SDA D21, SCL D22 | O sensor responde em 0x68? A gravidade dá ≈ 1 g? Qual o ruído e o bias do giro? |
| `teste_2_bmi323_spi` | BMI323 (módulo GY-601N1) | SPI — SCLK D18, MOSI D19, MISO D26, CS D25 | O chip responde (`CHIP_ID` 0x43)? A configuração é aceita? Qual o ruído? |
| `teste_3_bmi323_i2c` | BMI323 (módulo GY-601N1) | I2C — SCL D18, SDA D19 (+ CS D25, SA0 D26; VCC no D23; PS solto) | O chip responde por I2C em 0x68? Mesmas medidas do teste 2 |
| `teste_4_gy601n1_mcu` | Micro do GY-601N1 (ângulos prontos) | UART — TX/DA D16, RX/CL D17, PS D27 (3V3) | O micro responde? Chegam roll/pitch/yaw/temperatura? |
| `teste_5_motores_esc` | 2 ESCs + 2 motores BLDC | PWM 50 Hz (1000–2000 µs) — ESC1 D32, ESC2 D33 | Os ESCs armam? Cada motor gira? Qual o lado e o sentido? Em que pulso cada um parte? |

Cada pasta tem:
- o sketch `.ino` (código comentado passo a passo);
- um `README.md` com **ligação pino a pino, como rodar e o que esperar**;
- uma imagem `ligacao_teste_N_….png` com **somente os pinos daquele teste**.

## Como usar (receita geral)
1. **Monte só a ligação do teste** (veja o README e a imagem da pasta). Pinos de outros testes ficam soltos.
2. Arduino IDE → placa **ESP32 Dev Module** → porta COM do ESP32.
3. Abra o `.ino` da pasta → **Upload**.
4. Abra o **Monitor Serial a 115200 baud** ("Nova linha").
5. Leia o resultado. Os testes de sensor imprimem `[ OK ]` ou `[FALHOU]` por etapa (com a dica do que conferir); o teste dos motores é comandado por teclas.
6. `r` + ENTER repete os testes de sensor.

## O que cada teste de sensor confere (as 4 etapas em comum)
1. **Barramento / protocolo:** o chip aparece (endereço I2C ou `CHIP_ID` por SPI)?
2. **Identidade:** o registrador de identidade devolve o valor certo (MPU6050 = 0x68, BMI323 = 0x43)?
3. **Parado (~2–4 s):** 400 amostras com a viga **parada e nivelada** → **gravidade** (≈ 1 g), **ruído** (desvio-padrão do acelerômetro e do giro), **bias do giro** (vira offset) e **zero de roll/pitch**. Imprime uma linha `RESUMO_CSV,…` igual para os 3 sensores, para comparar.
4. **Ao vivo:** roll, pitch e giro com os offsets descontados; incline a viga e veja mudar.

Critérios de aprovação nas etapas de sensor: 400 amostras válidas · gravidade entre **0,95 e 1,05 g** · ruído do giro **< 0,5 °/s** · bias residual **< 0,2 °/s**.

## Regras de segurança (motores)
- **Sem hélice** em qualquer teste de motor.
- USB do ESP32 **antes** da bateria. Digite `0` + ENTER para parar.
- Os testes de sensor **não precisam de bateria, ESC nem motor**.

## Regras do módulo GY-601N1
- **PS solto** = o micro da placa fica quieto → o ESP32 lê o BMI323 por I2C ou SPI sem conflito (testes 2 e 3 e dado bruto).
- **PS em 3V3** = o micro manda os ângulos por UART (teste 4). O código do teste põe o D27 em 3V3 sozinho; nos outros testes o D27 fica como entrada (solto).
- O **VCC do módulo vai no D23** (o ESP32 liga e desliga); CS e SA0 são postos antes de ligar.

## Mapa de pinos (resumo)
| Pino ESP32 | Teste 1 | Teste 2 | Teste 3 | Teste 4 | Teste 5 |
|---|---|---|---|---|---|
| 3V3 / GND | MPU | GND do BMI | GND do BMI | GND do micro | GND dos ESCs |
| D23 | — | VCC do BMI | VCC do BMI | VCC do micro | — |
| D18 | — | SCLK | SCL | — | — |
| D19 | — | MOSI | SDA | — | — |
| D21 | SDA | — | — | — | — |
| D22 | SCL | — | — | — | — |
| D25 | — | CS | CS | CS | — |
| D26 | — | MISO | SA0 | — | — |
| D27 | — | solto | solto | PS | — |
| D16 | — | — | — | TX/DA | — |
| D17 | — | — | — | RX/CL | — |
| D32 | — | — | — | — | ESC 1 |
| D33 | — | — | — | — | ESC 2 |

