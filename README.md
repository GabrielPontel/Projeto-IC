# Projeto-IC

Repositório da Iniciação Científica (IFSP – Presidente Epitácio):
**Desenvolvimento de uma Aplicação Web do Controlador PID para Estabilização de um Motor Brushless via ESP32.**

A bancada é uma balança (viga) com 2 motores brushless, controlada por um ESP32 com PID em cascata, lendo a inclinação por sensor inercial (MPU6050 ou BMI323).

## Organização

| Pasta | Conteúdo |
|---|---|
| [`proposta/`](proposta) | Proposta de IC (PDF do SUAP) |
| [`testes_componentes/`](testes_componentes) | Testes isolados de cada componente: MPU6050 (I2C), BMI323 (SPI e I2C), micro do GY-601N1 (UART) e ESCs/motores (PWM) |

## Para que serve `testes_componentes/`

Antes de usar um componente no firmware principal, valide-o sozinho: se os fios estão certos, se o protocolo responde e se o sinal é bom. Se você for usar o **MPU6050**, o **BMI323 / GY-601N1** ou os **ESCs e motores**, é nessa pasta que está o teste correspondente, com o passo a passo de ligação e execução.

Veja o [README da pasta](testes_componentes/README.md) para a visão geral de cada teste.
