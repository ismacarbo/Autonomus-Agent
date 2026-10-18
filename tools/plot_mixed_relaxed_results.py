#!/usr/bin/env python3
"""Offline figures: measured endpoints and selected/diagnostic paths, no plant."""
from pathlib import Path
import csv
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
folder = root / 'documentation/analysis_20261018/experimental_relaxed'
fixture = root / 'simulator/src/tests/integration/data/mixed_20261018_101839_observed_scans.txt'
lines = iter(fixture.read_text().splitlines()); hits = []
for header in lines:
    t, x, y, yaw, ox, oy, count = map(float, header.split())
    for _ in range(int(count)):
        a, r = map(float, next(lines).split()); hits.append((ox+r*math.cos(a), oy+r*math.sin(a)))
def read(path):
    return [{k: float(v) for k,v in row.items()} for row in csv.DictReader(path.open())]
fig, ax = plt.subplots(figsize=(8, 7))
ax.scatter(*zip(*hits), s=2, color='.5', label='Endpoint misurati: ultime 5 scansioni')
ax.plot([0,1.3],[0,0],'k--',label='Reference persistente')
for index, path in enumerate(sorted((folder/'curves').glob('recorded_selected_*.csv'))):
    data = read(path);ax.plot([r['x'] for r in data],[r['y'] for r in data],lw=2,label=f'Segmento locale gate {index}')
    ax.scatter(data[-1]['x'],data[-1]['y'],marker='s',s=20)
for gate in read(folder/'curves/recorded_gates.csv'):
    dx,dy=-math.sin(gate['yaw'])*gate['width']/2,math.cos(gate['yaw'])*gate['width']/2
    ax.plot([gate['x']-dx,gate['x']+dx],[gate['y']-dy,gate['y']+dy],color='purple',alpha=.6)
ax.scatter([1.3],[0],marker='*',s=110,color='black');ax.set_aspect('equal');ax.grid(alpha=.2)
ax.set(xlabel='x [m]',ylabel='y [m]',xlim=(-.5,1.7),ylim=(-1,1.2),title='RELAXED: segmenti verificati sullo snapshot\nNessuna esecuzione fisica; perimetro arena non registrato')
ax.legend(fontsize=8);fig.tight_layout();fig.savefig(folder/'recorded_paths.png',dpi=160)
fig, ax = plt.subplots(figsize=(8,4))
for name, label in [('recorded_unconstrained_2','G2 geometrica, NON eseguibile'),('recorded_selected_2','Segmento selezionato')]:
    data=read(folder/'curves'/f'{name}.csv');arclength=[0.]
    for a,b in zip(data,data[1:]):arclength.append(arclength[-1]+math.hypot(a['x']-b['x'],a['y']-b['y']))
    ax.plot(arclength,[r['curvature'] for r in data],label=label)
for v,label,color in [(1.8,'Limite operativo','orange'),(math.tan(.52)/.18,'Limite meccanico del modello','red')]:
    ax.axhline(v,color=color,ls='--',label=label);ax.axhline(-v,color=color,ls='--')
ax.set(xlabel='Ascissa lungo curva [m]',ylabel='Curvatura [1/m]',title='Stessa posa e gate ricostruito 2: geometria vs segmento ammissibile');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(folder/'curvature_comparison.png',dpi=160)
