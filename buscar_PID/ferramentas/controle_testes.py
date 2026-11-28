#!/usr/bin/env python3
"""
Ponte USB <-> arquivos para os testes de PID da viga (dRehmFlight 2 motores).

- Abre o ESP32 (COM4, 500000 baud) e liga o log rapido ('l').
- Le comandos da pasta  controle/comandos.txt  (uma linha por comando) e manda ao ESP.
  Comandos especiais (nao vao ao ESP):
     !novo <nome>   comeca um novo teste -> dados/<nome>_<data>.csv
     !fim           fecha o teste atual
     !espera <s>    pausa o envio dos proximos comandos por <s> segundos
- Tudo que o ESP responde (menos LOG) vai para controle/saida.txt
- Linhas LOG (~100 Hz) vao para o CSV do teste aberto.

Rodar: clique duas vezes em  INICIAR_CONTROLE.bat   (fechar a janela = parar; desarma antes).
Feche o Monitor Serial da Arduino IDE antes.
"""
import traceback, csv, datetime, os, sys, time, collections

try:
    import serial
except ImportError:
    sys.exit("Falta pyserial: rode  pip install pyserial")

PORTA = sys.argv[1] if len(sys.argv) > 1 else "COM4"
AQUI = os.path.dirname(os.path.abspath(__file__))
CTRL = os.path.join(AQUI, "controle"); DADOS = os.path.join(AQUI, "dados")
os.makedirs(CTRL, exist_ok=True); os.makedirs(DADOS, exist_ok=True)
F_CMD = os.path.join(CTRL, "comandos.txt"); F_OUT = os.path.join(CTRL, "saida.txt")
open(F_CMD, "w").close()

COLS = ["tempo_s", "roll_graus", "pitch_graus", "giro_x_dps", "giro_y_dps", "giro_z_dps",
        "pid_roll", "pid_pitch", "pid_yaw", "motor1_us", "motor2_us", "Kp", "Ki", "Kd", "acelerador_us",
        "acc_x_g", "acc_y_g", "acc_z_g", "micros"]
COLS_STATS = ["tempo_s", "bodyAvg_us", "bodyMax_us", "estouros", "perAvg_us", "perMax_us", "perMin_us", "vibRms_g", "vibPeak_g", "alvo_us", "ciclos"]

def log(msg):
    linha = f"{datetime.datetime.now():%H:%M:%S.%f}"[:-3] + " " + msg
    print(linha)
    with open(F_OUT, "a", encoding="utf-8") as f:
        f.write(linha + "\n")

def br(x): return x.replace(".", ",")

def main():
    try:
        ser = serial.Serial(PORTA, 500000, timeout=0.05)
    except serial.SerialException as e:
        input(f"Nao abriu {PORTA}: {e}\nFeche o Monitor Serial e tente de novo. [Enter]"); return
    time.sleep(2.0)                         # ESP reinicia ao abrir a porta; espera calibrar
    log(f"PONTE ABERTA em {PORTA}. Aguardando comandos em controle/comandos.txt")

    pos = 0; fila = collections.deque(); pausa_ate = 0
    arq = None; w = None; arq2 = None; w2 = None; ts0 = 0; t0 = None; n = 0; buf = b""; ult_log = 0; ult_l = 0; log_ligado = False
    try:
        while True:
            # novos comandos do arquivo
            with open(F_CMD, "r", encoding="utf-8") as f:
                f.seek(pos); novo = f.read(); pos = f.tell()
            fila.extend(l.strip() for l in novo.splitlines() if l.strip())
            while fila and time.time() >= pausa_ate:
                cmd = fila.popleft()
                if cmd.startswith("!novo"):
                    if arq:
                        arq.close(); arq2.close(); log(f"TESTE FECHADO ({n} amostras)")
                    nome = (cmd.split(maxsplit=1)[1:] or ["teste"])[0].replace(" ", "_")
                    caminho = os.path.join(DADOS, f"{nome}_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv")
                    arq = open(caminho, "w", newline="", encoding="utf-8")
                    w = csv.writer(arq, delimiter=";"); w.writerow(COLS); t0 = None; n = 0
                    arq2 = open(caminho.replace(".csv", "_stats.csv"), "w", newline="", encoding="utf-8")
                    w2 = csv.writer(arq2, delimiter=";"); w2.writerow(COLS_STATS); ts0 = time.time()
                    log(f"TESTE NOVO: {caminho}")
                elif cmd == "!fim":
                    if arq:
                        arq.close(); arq2.close(); log(f"TESTE FECHADO ({n} amostras)")
                    arq = None; w = None
                elif cmd.startswith("!espera"):
                    pausa_ate = time.time() + float(cmd.split()[1])
                else:
                    ser.write((cmd + "\n").encode()); log(f">> {cmd}")
            # dados do ESP
            buf += ser.read(4096)
            *linhas, buf = buf.split(b"\n")
            for raw in linhas:
              try:
                s = raw.decode(errors="ignore").strip()
                if not s: continue
                c = s.split(",")
                if c[0] == "LOG" and len(c) >= 18:
                    ult_log = time.time(); log_ligado = True
                    if w:
                        ms = int(c[1]); t0 = ms if t0 is None else t0
                        # LOG,ms,roll,pitch,gx,gy,gz,pidR,pidP,pidY,m1,m2,m3,m4,Kp,Ki,Kd,thr
                        w.writerow([br(f"{(ms - t0)/1000:.3f}")] + [br(v) for v in c[2:10]] +
                                   [c[10], c[11], br(c[14]), br(c[15]), br(c[16]), c[17]] +
                                   ([br(c[18]), br(c[19]), br(c[20]), c[21]] if len(c) >= 22 else ["", "", "", ""]))
                        n += 1
                        if n % 200 == 0: arq.flush()
                elif c[0] == "STATS":
                    if arq and len(c) >= 11: w2.writerow([br(f"{time.time()-ts0:.2f}")] + [br(v) for v in c[1:11]]); arq2.flush()
                    # loop rodando: se o log rapido nao chega, liga (no maximo 1x a cada 4 s)
                    if time.time() - ult_log > 3 and time.time() - ult_l > 4:
                        ser.write(b"l"); ult_l = time.time()
                else:
                    log("ESP: " + s)
              except Exception as e:
                log(f"(linha ignorada: {e})")
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback; log("ERRO:\n" + traceback.format_exc())
    finally:
        try: ser.write(b"d"); time.sleep(0.1); ser.close()
        except Exception: pass
        if arq: arq.close()
        log("PONTE FECHADA (desarmado)")

if __name__ == "__main__":
    main()
