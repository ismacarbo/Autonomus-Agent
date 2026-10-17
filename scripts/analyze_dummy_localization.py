#!/usr/bin/env python3
"""Quantitative, read-only comparison of GUI hardware exports.

Requires numpy and matplotlib. Figures describe the recorded software version,
not later fixes; SLAM/odometry discrepancies are not ground-truth errors.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

COLORS = ['#2563eb', '#d97706', '#059669']


def load_run(path, label):
    data = json.loads(path.read_text())
    h = data['history']
    if 'observe_only' not in data.get('slam_backend', {}).get('status', ''):
        raise ValueError(f'{path}: these figure captions require documented SLAM observe-only mode')
    if any(s['slam_correction_accepted'] > .5 for s in h):
        raise ValueError(f'{path}: feedback was applied; use separate captions/analysis')
    accepted = next((i for i, s in enumerate(h) if s['start_matching_enabled'] > .5 and
                     s['start_matching_complete'] > .5 and s['start_matching_accepted'] > .5), None)
    motion = next((i for i, s in enumerate(h) if s['pwm_left'] or s['pwm_right']), None)
    if accepted is None or motion is None:
        raise ValueError(f'{path}: cannot compare a run without accepted start and commanded motion')
    first_gate = next((i for i in range(1, len(h)) if h[i]['passed_gates'] > h[i-1]['passed_gates']), None)
    if first_gate is None:
        raise ValueError(f'{path}: no first crossing; use a censored-outcome analysis instead')
    t = np.array([s['controller_encoder_ms'] for s in h]) / 1000
    if np.any(np.diff(t) < 0):
        raise ValueError(f'{path}: MCU reset/wrap; cannot use this clock without segmentation')
    t -= t[motion]
    a = h[accepted]
    c, s = math.cos(a['yaw']), math.sin(a['yaw'])
    positions = np.array([[r['position_x'], r['position_y']] for r in h])
    positions = (positions - [a['position_x'], a['position_y']]) @ np.array([[c, -s], [s, c]])
    mpc = [i for i in range(motion, first_gate) if h[i]['control_source'] == 2]
    if not mpc:
        raise ValueError(f'{path}: no GATE_MPC before first crossing')
    pre = h[:accepted]
    valid = np.array([r['slam_pose_valid'] > .5 for r in h])
    gate = h[first_gate-1]
    dx = h[first_gate]['position_x'] - gate['locked_gate_crossing_x']
    dy = h[first_gate]['position_y'] - gate['locked_gate_crossing_y']
    angle = gate['locked_gate_heading_rad']
    yaw_error = math.degrees(math.atan2(math.sin(h[first_gate]['yaw']-angle), math.cos(h[first_gate]['yaw']-angle)))
    m = dict(run=label, snapshot=path.stem, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        firmware=f"{int(a['controller_firmware_major'])}.{int(a['controller_firmware_minor'])}",
        calibration=data['scene']['calibration_profile_version'],
        initial_dx_cm=100*(a['position_x']-h[0]['position_x']),
        initial_dy_cm=100*(a['position_y']-h[0]['position_y']),
        initial_yaw_deg=math.degrees(a['yaw']-h[0]['yaw']),
        start_confidence=a['start_matching_confidence'],
        pwm_nonzero_before_accept=sum(any(r[k] for k in ('pwm_left','pwm_right','controller_pwm_left','controller_pwm_right','controller_target_pwm_left','controller_target_pwm_right')) for r in pre),
        encoder_ticks_before_accept=sum(abs(r['controller_left_encoder_delta'])+abs(r['controller_right_encoder_delta']) for r in pre),
        slam_valid_samples=int(valid.sum()), total_samples=len(h),
        slam_valid_percent=100*valid.mean(),
        slam_feedback_samples=sum(r['slam_correction_accepted'] > .5 for r in h),
        graph_nodes=int(h[-1]['slam_graph_nodes']), map_updates=int(h[-1]['slam_map_updates']),
        gate_mpc_samples=len(mpc), gate_mpc_with_reference=sum(h[i]['planner_has_reference'] > .5 for i in mpc),
        first_mpc_s=float(t[mpc[0]]), first_gate_s=float(t[first_gate]),
        crossing_lateral_cm=100*(-math.sin(angle)*dx+math.cos(angle)*dy),
        crossing_heading_deg=yaw_error, gates_counted=int(h[-1]['passed_gates']),
        slam_difference_at_gate_m=h[first_gate]['slam_position_innovation_m'],
        slam_difference_max_m=max(r['slam_position_innovation_m'] for r in h if r['slam_pose_valid']),
        slam_difference_median_m=float(np.median([r['slam_position_innovation_m'] for r in h if r['slam_pose_valid']])),
        external_ground_truth_samples=sum(r['external_pose_valid'] > .5 for r in h))
    m['initial_translation_cm'] = math.hypot(m['initial_dx_cm'], m['initial_dy_cm'])
    return dict(h=h, metrics=m, time=t, positions=positions, valid=valid, motion=motion, gate=first_gate, mpc=mpc)


def bar(ax, values, labels, title, ylabel, fmt='.2f', ylim=None):
    bars = ax.bar(labels, values, color=COLORS[:len(values)], width=.58)
    for b, value in zip(bars, values):
        ax.annotate(format(value, fmt), (b.get_x()+b.get_width()/2, b.get_height()),
                    xytext=(0, 4 if value >= 0 else -13), textcoords='offset points', ha='center', fontsize=10)
    ax.set(title=title, ylabel=ylabel)
    if ylim: ax.set_ylim(*ylim)
    ax.grid(axis='y', alpha=.18)
    ax.set_axisbelow(True)


def figures(runs, out):
    labels = [r['metrics']['run'] for r in runs]
    metrics = [r['metrics'] for r in runs]
    plt.rcParams.update({'font.size': 10, 'axes.titlesize': 11, 'figure.titlesize': 16,
                         'pdf.fonttype': 42, 'savefig.facecolor': 'white'})
    figs = []
    fig, ax = plt.subplots(2, 3, figsize=(15, 9))
    fig.subplots_adjust(top=.86, bottom=.13, hspace=.43, wspace=.34)
    fig.suptitle('Start matching + SLAM: evidenze di funzionamento', y=.97)
    fig.text(.5,.925,'Tre dummy del 14/10/2026 · stesso allestimento dichiarato · dati precedenti alle nuove correzioni',ha='center',color='#475569')
    ax[0,0].add_patch(plt.Circle((0,0), 8, fill=False, ls='--', color='#64748b', label='Tolleranza dummy 8 cm'))
    for r, color in zip(metrics,COLORS):
        ax[0,0].scatter(r['initial_dx_cm'],r['initial_dy_cm'],s=75,color=color)
        ax[0,0].annotate(r['run'],(r['initial_dx_cm'],r['initial_dy_cm']),
            xytext={'A':(8,-11),'B':(6,5),'C':(-15,7)}[r['run']],textcoords='offset points')
    ax[0,0].set(xlim=(-9,9),ylim=(-9,9),aspect='equal',title='Posa iniziale: 3/3 accettate',xlabel='Offset x stimato [cm]',ylabel='Offset y stimato [cm]')
    ax[0,0].legend(fontsize=8,loc='upper right');ax[0,0].grid(alpha=.18)
    bar(ax[0,1],[r['initial_yaw_deg'] for r in metrics],labels,'Offset angolare accettato','Yaw stimato [°]',ylim=(-10,10))
    ax[0,1].axhline(8,color='#64748b',ls='--');ax[0,1].axhline(-8,color='#64748b',ls='--')
    bar(ax[0,2],[r['start_confidence'] for r in metrics],labels,'Confidenza dello start matcher','Indice del matcher (non probabilità)',fmt='.3f',ylim=(0,1.08))
    bar(ax[1,0],[r['slam_valid_percent'] for r in metrics],labels,'Disponibilità della posa SLAM','Campioni validi / campioni esportati [%]',fmt='.2f',ylim=(0,110))
    for i,r in enumerate(metrics):ax[1,0].text(i,50,f"{r['slam_valid_samples']}/{r['total_samples']}",ha='center',color='white',weight='bold')
    for run,color,label in zip(runs,COLORS,labels):
        ax[1,1].plot(run['time'],[s['slam_graph_nodes'] for s in run['h']],color=color,label=label)
    ax[1,1].set(title='Crescita del grafo SLAM',xlabel='Tempo MCU dal primo comando [s]',ylabel='Nodi del grafo');ax[1,1].legend();ax[1,1].grid(alpha=.18)
    bar(ax[1,2],[100*r['gate_mpc_with_reference']/r['gate_mpc_samples'] for r in metrics],labels,'Primo gate: MPC con riferimento','Campioni MPC con riferimento [%]',fmt='.0f',ylim=(0,115))
    for i,r in enumerate(metrics):ax[1,2].text(i,50,f"{r['gate_mpc_with_reference']}/{r['gate_mpc_samples']}",ha='center',color='white',weight='bold')
    pwm=sum(r['pwm_nonzero_before_accept'] for r in metrics);ticks=sum(r['encoder_ticks_before_accept'] for r in metrics)
    fig.text(.05,.05,f'Prima dell’accettazione: {pwm} campioni con PWM non nullo; {ticks:g} tick encoder. SLAM observe-only: 0 correzioni applicate alla navigazione.\nDisponibilità e crescita del grafo dimostrano attività della pipeline; la precisione metrica richiede ground truth indipendente.',fontsize=10)
    figs.append(('01_pipeline_function',fig))

    fig,ax=plt.subplots(2,2,figsize=(13,10));fig.subplots_adjust(top=.87,bottom=.14,hspace=.39,wspace=.30)
    fig.suptitle('Navigazione fino al primo gate conteggiato',y=.97)
    fig.text(.5,.925,'Allineamento solo alla posa iniziale accettata · nessun fit delle traiettorie',ha='center',color='#475569')
    for run,color,label in zip(runs,COLORS,labels):
        segment=run['positions'][run['motion']:run['gate']+1]
        ax[0,0].plot(segment[:,0],segment[:,1],color=color,label=label,lw=2)
        ax[0,0].scatter(*segment[-1],color=color,s=40)
    ax[0,0].set(title='Traiettorie stimate',xlabel='Avanti dallo start [m]',ylabel='Sinistra dallo start [m]',aspect='equal');ax[0,0].legend();ax[0,0].grid(alpha=.18)
    for i,(r,color) in enumerate(zip(metrics,COLORS)):
        ax[0,1].barh(i,r['first_mpc_s'],color='#cbd5e1')
        ax[0,1].barh(i,r['first_gate_s']-r['first_mpc_s'],left=r['first_mpc_s'],color=color)
        ax[0,1].text(r['first_gate_s']+.2,i,f"{r['first_gate_s']:.2f} s",va='center')
    ax[0,1].set(yticks=range(len(labels)),yticklabels=labels,xlim=(0,23),ylim=(-.6,3.0),title='Ricerca → avvicinamento → primo passaggio',xlabel='Tempo MCU dal primo comando [s]')
    ax[0,1].text(.02,.97,'Grigio: prima del primo MPC\nColore: dal primo MPC al passaggio',transform=ax[0,1].transAxes,va='top',fontsize=9)
    bar(ax[1,0],[abs(r['crossing_lateral_cm']) for r in metrics],labels,'Scarto laterale stimato al passaggio','|Scarto dal gate bloccato| [cm]',ylim=(0,6.4))
    bar(ax[1,1],[abs(r['crossing_heading_deg']) for r in metrics],labels,'Disallineamento stimato al passaggio','|Yaw rispetto al gate bloccato| [°]',ylim=(0,32))
    fig.text(.05,.05,'A/B agganciano un target in una zona simile; C devia durante la ricerca e aggancia un target diverso nella mappa stimata.\nI tre tempi non sono una misura di ripetibilità dello stesso gate identificato fisicamente. Nessun campione ArUco disponibile.',fontsize=10)
    figs.append(('02_gate_comparison',fig))

    fig,ax=plt.subplots(2,2,figsize=(13,9));fig.subplots_adjust(top=.85,bottom=.14,hspace=.44,wspace=.30)
    fig.suptitle('SLAM e odometria: confronto quantitativo tra stime',y=.97)
    fig.text(.5,.915,'Differenze riportate per la stessa scan · non errori rispetto alla ground truth',ha='center',color='#475569')
    for run,color,label in zip(runs,COLORS,labels):
        mask=run['valid'] & (run['time']>=0)
        t=run['time'][mask];h=run['h']
        ax[0,0].plot(t,np.array([s['slam_position_innovation_m'] for s in h])[mask],color=color,label=label)
        ax[0,1].plot(t,np.array([s['slam_yaw_innovation_deg'] for s in h])[mask],color=color,label=label)
    ax[0,0].set(title='Differenza di posizione',xlabel='Tempo MCU dal primo comando [s]',ylabel='Norma della differenza [m]')
    ax[0,1].set(title='Differenza di orientamento',xlabel='Tempo MCU dal primo comando [s]',ylabel='Differenza angolare [°]')
    for a in ax[0]:a.grid(alpha=.18);a.legend()
    bar(ax[1,0],[r['slam_difference_at_gate_m'] for r in metrics],labels,'Differenza alla prima traversata stimata','SLAM / odometria [m]',fmt='.3f',ylim=(0,.75))
    x=np.arange(len(labels));width=.32
    ax[1,1].bar(x-width/2,[r['slam_difference_median_m'] for r in metrics],width,label='Mediana',color='#64748b')
    ax[1,1].bar(x+width/2,[r['slam_difference_max_m'] for r in metrics],width,label='Massimo',color=COLORS)
    ax[1,1].set(xticks=x,xticklabels=labels,title='Intera acquisizione: durate diverse',ylabel='SLAM / odometria [m]');ax[1,1].legend();ax[1,1].grid(axis='y',alpha=.18)
    fig.text(.05,.05,'Le discrepanze rimangono visibili: questi dati dimostrano start matching accettato, SLAM attivo e MPC con riferimento.\nNon dimostrano accuratezza assoluta né l’effetto delle correzioni software successive. SLAM non modifica la navigazione in queste run.',fontsize=10)
    figs.append(('03_slam_odometry_comparison',fig))
    with PdfPages(out/'quantitative_comparison.pdf') as pdf:
        for name,fig in figs:
            fig.savefig(out/(name+'.png'),dpi=180)
            fig.savefig(out/(name+'.pdf'))
            pdf.savefig(fig)
            plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports',nargs='+',type=Path)
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args()
    if len(args.reports)!=3:parser.error('provide the three dummy JSON exports')
    runs=[load_run(path,label) for path,label in zip(args.reports,['A','B','C'])]
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    # Original inputs never overwritten, including a mistakenly chosen output path.
    input_paths={p.resolve() for p in args.reports}
    if (out/'quantitative_metrics.json').resolve() in input_paths:raise ValueError('output overlaps input')
    metrics=[r['metrics'] for r in runs]
    (out/'quantitative_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    with (out/'quantitative_metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(metrics[0]));writer.writeheader();writer.writerows(metrics)
    figures(runs,out)
    print(json.dumps({'output_dir':str(out),'runs':metrics},indent=2))

if __name__=='__main__':main()
