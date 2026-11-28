#!/usr/bin/env python3
# Executor de testes de sintonia (via ponte controle/comandos.txt). Uso: python3 rodar_teste_pid.py Kp Ki Kd [hold_s] [thr_final]
import sys,time,re,glob,csv,statistics as st,os
Kp,Ki,Kd=sys.argv[1:4]; hold=float(sys.argv[4]) if len(sys.argv)>4 else 16; thr=int(sys.argv[5]) if len(sys.argv)>5 else 1200
base=os.path.dirname(os.path.abspath(__file__)); C=base+'/controle/comandos.txt'; S=base+'/controle/saida.txt'
def send(x): open(C,'a').write(x+'\n')
def roll():
    send('p'); time.sleep(0.8)
    try:
        t=open(S,'rb').read()[-900:].decode('latin1')
        m=re.findall(r'STATE:.*?roll=(-?[\d.]+)',t); return float(m[-1]) if m else None
    except Exception: return None
tag=f"zn_P{Kp}_I{Ki}_D{Kd}_thr{thr}"
send(f'#L{max(thr+50,1250)}'); send(f'#P{Kp}'); send(f'#I{Ki}'); send(f'#D{Kd}'); time.sleep(1)
send('!novo '+tag); time.sleep(1); send('#T1000'); time.sleep(0.8); send('a'); time.sleep(1.5)
send('#T1150'); time.sleep(2.5); send(f'#T{thr}')
t0=time.time(); abort=None
while time.time()-t0<hold:
    r=roll()
    if r is not None and abs(r)>38: abort=f'roll {r}'; break
send('#T1000'); time.sleep(0.5); send('d'); time.sleep(0.5); send('!fim'); time.sleep(1.5)
f=sorted(x for x in glob.glob(base+f'/dados/{tag}_*.csv') if not x.endswith('_stats.csv'))[-1]
rows=list(csv.DictReader(open(f,encoding='utf-8-sig'),delimiter=';'))
g=lambda k:[float(x[k].replace(',','.')) for x in rows]
t=g('tempo_s'); ro=g('roll_graus'); th=g('acelerador_us'); m1=g('motor1_us'); m2=g('motor2_us')
ix=[k for k in range(len(t)) if th[k]>=thr]
if len(ix)<50: print(tag,'sem dados suficientes'); sys.exit()
ts=[t[k] for k in ix]; rs=[ro[k] for k in ix]; T=ts[-1]-ts[0]
# janela de analise: depois de 4 s a thr final
w=[(a,b) for a,b in zip(ts,rs) if a-ts[0]>=4]; wt=[a for a,b in w]; wr=[b for a,b in w]
mean=st.mean(wr); sd=st.pstdev(wr); p2p=max(wr)-min(wr)
h=len(wr)//2; p1=max(wr[:h])-min(wr[:h]); p2=max(wr[h:])-min(wr[h:])
cr=[wt[i] for i in range(1,len(wr)) if (wr[i-1]-mean)<0<=(wr[i]-mean)]
per=(st.mean([cr[i+1]-cr[i] for i in range(len(cr)-1)]) if len(cr)>2 else float('nan'))
ratio=p2/p1 if p1>0 else float('nan')
if abort: v='INSTAVEL (abortado: '+abort+')'
elif p2p<3: v='amortecido/estavel'
elif ratio>1.25: v='oscilacao CRESCENTE'
elif ratio>0.8: v='oscilacao SUSTENTADA (candidata a Ku)'
else: v='oscilacao amortecendo'
lin=f"| {time.strftime('%H:%M')} | {Kp} | {Ki} | {Kd} | {thr} | {mean:.1f} | {p2p:.1f} | {p1:.1f} -> {p2:.1f} | {per:.2f} | {st.mean([m2[k]-m1[k] for k in ix]):.0f} | {v} | {os.path.basename(f)} |"
reg=base+'/dados/REGISTRO_SINTONIA.md'
if not os.path.exists(reg):
    open(reg,'w',encoding='utf-8').write("# Registro de sintonia da MPU (Z-N, fase 1 e seguintes)\n\n| hora | Kp | Ki | Kd | acel (us) | roll medio (graus) | pico-a-pico (graus) | p2p 1a -> 2a metade | periodo (s) | dif. m2-m1 (us) | veredito | arquivo |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n")
open(reg,'a',encoding='utf-8').write(lin+'\n'); print(lin)
