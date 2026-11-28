# -*- coding: utf-8 -*-
"""
Comparativo final: MPU6050 I2C x BMI323 I2C x BMI323 SPI x BMI323 SPI + filtro.
Gera 4 graficos (PNG, 200 dpi) em resultados/graficos/ :
  1_jitter_angulo_boxplot.png   boxplot do erro de roll em regime (oscilacao residual)
  2_jitter_angulo_colunas.png   erro medio x pior caso do roll
  3_jitter_tempo_colunas.png    periodo medio x pior caso do laco de controle (+ estouros)
  4_trepidacao_colunas.png      vibracao (RMS da aceleracao Z), motores desligados x ligados
Usa, em cada pasta de resultados/dados/ (1_MPU6050_I2C, 2_BMI323_I2C, 3_BMI323_SPI, 4_BMI323_SPI_filtro), o CSV *3min_final* mais recente
e o respectivo *_stats.csv. Requisitos: pip install numpy scipy matplotlib

DEFINICOES
  Regime       : do 60o segundo apos armar ate o fim (descarta a saida do batente). 'Parado' = 5 s apos ligar ate 5 s antes de armar.
  Erro de roll : roll medido - setpoint (0 graus). Desvio padrao = sigma. Pior caso = maior |roll| do regime.
  Periodo      : tempo entre o inicio de dois ciclos do laco (alvo 500 us = 2 kHz). Medio = media dos 'perAvg' de cada segundo;
                 pior caso = maior 'perMax' do regime.
  Estouro      : ciclo cujo tempo de trabalho (bodyUs, medido no ESP32 antes da espera do loopRate) e >= 4000 us (trava de 4 ms).
                 O numero mostrado e a media de estouros POR SEGUNDO no regime.
  Vibracao     : RMS do desvio padrao da aceleracao Z, filtrada em 2-40 Hz (tira a gravidade e iguala a banda entre os sensores,
                 pois o log tem 83 Hz no I2C e 125 Hz no SPI). Config sem aceleracao registrada (SPI + filtro) fica de fora.
"""
import csv, glob, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from scipy import signal

AQUI = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.join(AQUI, "resultados", "dados")
OUT = os.path.join(AQUI, "resultados", "graficos"); os.makedirs(OUT, exist_ok=True)
CFG = [("MPU6050\nI2C", "1_MPU6050_I2C", "#4C78A8"), ("BMI323\nI2C", "2_BMI323_I2C", "#F58518"),
       ("BMI323\nSPI", "3_BMI323_SPI", "#54A24B"), ("BMI323 SPI\n+ filtro", "4_BMI323_SPI_filtro", "#B279A2")]
ALVO_US, LIMITE_ESTOURO_US, BANDA_HZ, SKIP_S = 500, 4000, (2, 40), 60
plt.rcParams.update({"font.size": 15, "axes.titlesize": 18, "axes.labelsize": 15, "xtick.labelsize": 14, "ytick.labelsize": 14,
                     "legend.fontsize": 13, "axes.grid": True, "grid.alpha": .3, "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 200})
def vc(v, n=0): return f"{v:,.{n}f}".replace(",", "X").replace(".", ",").replace("X", ".")   # 1.234,5
def load(p):
    r = list(csv.reader(open(p, encoding="utf-8-sig"), delimiter=";")); h = r[0]
    d = np.array([[float(x.replace(",", ".")) for x in l] for l in r[1:] if l]); return {n: d[:, i] for i, n in enumerate(h)}
def passa_faixa(x, fs): return signal.sosfiltfilt(signal.butter(2, BANDA_HZ, "bandpass", fs=fs, output="sos"), x)

D = []
for nome, pasta, cor in CFG:
    f = [x for x in sorted(glob.glob(f"{BASE}/{pasta}/*3min_final*.csv")) if not x.endswith("_stats.csv")][-1]
    d, s = load(f), load(f.replace(".csv", "_stats.csv")); t = d["tempo_s"]
    arm = np.where(d["acelerador_us"] > 1100)[0]; t0, t1 = t[arm[0]], t[arm[-1]]
    reg = (t >= t0 + SKIP_S) & (t <= t1 - 1); sreg = (s["tempo_s"] >= t0 + SKIP_S) & (s["tempo_s"] <= t1 - 1)
    fs = round(1 / np.median(np.diff(t))); tem_acc = bool(np.std(d["acc_z_g"]) > 1e-4); rms = {}
    if tem_acc:
        for tag, (a, b) in dict(parado=(5, t0 - 5), armado=(t0 + SKIP_S, t1 - 1)).items():
            m = (t >= a) & (t <= b); tt = np.arange(t[m][0], t[m][-1], 1 / fs)
            rms[tag] = np.std(passa_faixa(np.interp(tt, t[m], d["acc_z_g"][m]), fs)) * 1e3     # mg
    D.append(dict(nome=nome, cor=cor, arquivo=os.path.basename(f), roll=d["roll_graus"][reg], per_medio=s["perAvg_us"][sreg].mean(),
                  per_pior=s["perMax_us"][sreg].max(), estouros_s=s["estouros"][sreg].mean(), tem_acc=tem_acc, rms=rms))
    print(nome.replace("\n", " "), "->", os.path.basename(f))
x = np.arange(len(D)); rotulos = [c["nome"] for c in D]

# ---------- 1) Jitter do angulo: boxplot
fig, ax = plt.subplots(figsize=(11, 7.5))
bp = ax.boxplot([c["roll"] for c in D], positions=x, widths=.6, patch_artist=True, whis=(1, 99), showfliers=False, medianprops=dict(color="k", lw=2.5))
for p, c in zip(bp["boxes"], D): p.set_facecolor(c["cor"]); p.set_alpha(.7)
ax.axhline(0, color="k", lw=1)
for i, c in enumerate(D): ax.text(i, 0.66, f"σ = {vc(np.std(c['roll']), 3)}°", ha="center", fontsize=14, fontweight="bold")
ax.set_xticks(x); ax.set_xticklabels(rotulos); ax.set_ylim(-0.75, 0.78); ax.set_ylabel("Erro de roll em regime (graus)")
ax.set_title("Jitter do ângulo: oscilação residual da viga\n(caixa = metade das medidas; fios = percentis 1 a 99)")
fig.tight_layout(); fig.savefig(f"{OUT}/1_jitter_angulo_boxplot.png"); plt.close(fig)

# ---------- 2) Jitter do angulo: erro medio x pior caso
fig, ax = plt.subplots(figsize=(11, 7.5)); w = .36
med = [np.mean(np.abs(c["roll"])) for c in D]; pior = [np.max(np.abs(c["roll"])) for c in D]
b1 = ax.bar(x - w / 2, med, w, color="#9E9E9E"); b2 = ax.bar(x + w / 2, pior, w, color=[c["cor"] for c in D])
for b, v in list(zip(b1, med)) + list(zip(b2, pior)): ax.text(b.get_x() + b.get_width() / 2, v + .015, vc(v, 2) + "°", ha="center", fontsize=14)
ax.set_xticks(x); ax.set_xticklabels(rotulos); ax.set_ylim(0, 1.15); ax.set_ylabel("Erro de roll (graus)")
ax.set_title("Jitter do ângulo: erro médio × pior caso")
ax.legend(handles=[Patch(color="#9E9E9E", label="Erro médio (média de |roll|)"), Patch(facecolor="white", edgecolor="k", label="Pior caso (maior |roll|): barra colorida")], loc="upper left")
fig.tight_layout(); fig.savefig(f"{OUT}/2_jitter_angulo_colunas.png"); plt.close(fig)

# ---------- 3) Jitter do tempo do laco: periodo medio x pior caso (+ estouros)
fig, ax = plt.subplots(figsize=(11, 7.5)); w = .36
pm = [c["per_medio"] for c in D]; pp = [c["per_pior"] for c in D]
b1 = ax.bar(x - w / 2, pm, w, color="#9E9E9E"); b2 = ax.bar(x + w / 2, pp, w, color=[c["cor"] for c in D])
for b, v in list(zip(b1, pm)) + list(zip(b2, pp)): ax.text(b.get_x() + b.get_width() / 2, v + 70, vc(v), ha="center", fontsize=14)
ax.axhline(ALVO_US, color="#2E7D32", ls="--", lw=2)
ax.axhline(LIMITE_ESTOURO_US, color="#C62828", ls="--", lw=2)
for i, c in enumerate(D): ax.text(i, -950, f"{vc(c['estouros_s'])} estouros/s", ha="center", fontsize=14, fontweight="bold", color="#C62828" if c["estouros_s"] > 0 else "#2E7D32")
ax.set_xticks(x); ax.set_xticklabels(rotulos); ax.set_ylim(0, 5600); ax.set_ylabel("Período do laço de controle (µs)")
ax.set_title("Jitter do tempo do laço: período médio × pior caso")
ax.legend(handles=[Patch(color="#9E9E9E", label="Período médio"), Patch(facecolor="white", edgecolor="k", label="Pior caso (maior período): barra colorida"),
                   plt.Line2D([0], [0], color="#2E7D32", ls="--", lw=2, label=f"Alvo: {ALVO_US} µs (2 kHz)"), plt.Line2D([0], [0], color="#C62828", ls="--", lw=2, label=f"Limite de estouro: {vc(LIMITE_ESTOURO_US)} µs")], loc="upper right", fontsize=12)
fig.tight_layout(); fig.subplots_adjust(bottom=.22); fig.savefig(f"{OUT}/3_jitter_tempo_colunas.png"); plt.close(fig)

# ---------- 4) Trepidacao da estrutura
V = [c for c in D if c["tem_acc"]]; xv = np.arange(len(V)); fig, ax = plt.subplots(figsize=(10, 7.5)); w = .36
p = [c["rms"]["parado"] for c in V]; a = [c["rms"]["armado"] for c in V]
b1 = ax.bar(xv - w / 2, p, w, color="#9E9E9E"); b2 = ax.bar(xv + w / 2, a, w, color=[c["cor"] for c in V])
for b, v in list(zip(b1, p)) + list(zip(b2, a)): ax.text(b.get_x() + b.get_width() / 2, v + 1.2, vc(v, 1), ha="center", fontsize=14)
ax.set_xticks(xv); ax.set_xticklabels([c["nome"] for c in V]); ax.set_ylabel("Vibração: RMS da aceleração Z (mg)"); ax.set_title("Trepidação da estrutura")
ax.legend(handles=[Patch(color="#9E9E9E", label="Motores desligados"), Patch(facecolor="white", edgecolor="k", label="Motores ligados (PID, 1150 µs): barra colorida")], loc="upper left")
fig.tight_layout(); fig.subplots_adjust(bottom=.2)
nota = f"Banda {BANDA_HZ[0]}–{BANDA_HZ[1]} Hz." + ("" if len(V) == len(D) else " BMI323 SPI + filtro: aceleração não registrada.")
fig.text(.5, .02, nota, ha="center", fontsize=11.5, color="#555"); fig.savefig(f"{OUT}/4_trepidacao_colunas.png"); plt.close(fig)
print("OK ->", OUT)
