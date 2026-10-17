"""Build the Italian colleague recap from immutable archived runs and SLAM replay.

Run with NumPy, Matplotlib, Pillow and ReportLab. No robot connection is used.
"""
from pathlib import Path
import csv
import hashlib
import json
import math
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
from matplotlib.transforms import Affine2D
from matplotlib.lines import Line2D
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import Paragraph, Table, TableStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'documentation/analysis_20261014/recap_collega'
FIG = OUT / 'figure'
DATA = OUT / 'data'
COLORS = ['#147D92', '#D47929', '#7355A2']
NAVY, GREY, RED = '#16334D', '#5E6E7A', '#B33C46'
SUFFIXES = ['20261014_001926_149', '20261014_002233_702', '20261014_002548_405']
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.labelcolor': NAVY,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.titleweight': 'bold', 'axes.titlesize': 11, 'axes.titlepad': 12,
                     'axes.edgecolor': '#BCC6CC', 'grid.color': '#E1E6EA', 'figure.facecolor': 'white',
                     'pdf.fonttype': 42, 'savefig.facecolor': 'white'})


def wrap(x):
    return np.arctan2(np.sin(x), np.cos(x))


def number(x, digits=2):
    return f'{x:.{digits}f}'.replace('.', ',')


def save(fig, name):
    fig.savefig(FIG / f'{name}.png', dpi=210, bbox_inches='tight', pad_inches=.12)
    fig.savefig(FIG / f'{name}.pdf', bbox_inches='tight', pad_inches=.12)
    plt.close(fig)
    return FIG / f'{name}.png'


def xy(sample, prefix='position_'):
    return np.array([sample[prefix+'x'], sample[prefix+'y']])


def load_run(label, suffix, color):
    path = ROOT / f'reports/thesis_hardware_unstructured_autonomous_lidar_exploration_gui_snapshot_{suffix}.json'
    d = json.loads(path.read_text())
    h = d['history']
    a = next(i for i, s in enumerate(h) if s['start_matching_accepted'])
    move = next(i for i, s in enumerate(h) if s['pwm_left'] or s['pwm_right'])
    t = (np.array([s['controller_encoder_ms'] for s in h]) - h[move]['controller_encoder_ms']) / 1000
    pos = np.array([xy(s) for s in h])
    events = []
    for i in range(1, len(h)):
        if h[i]['passed_gates'] <= h[i-1]['passed_gates']:
            continue
        prior = h[i-1]
        center = xy(prior, 'locked_gate_crossing_')
        angle = prior['locked_gate_heading_rad']
        n = np.array([np.cos(angle), np.sin(angle)])
        tangent = np.array([-n[1], n[0]])
        signed = (pos - center) @ n
        last_event = events[-1]['index'] if events else a
        candidates = [j for j in range(last_event+1, i+1) if signed[j-1] <= 0 < signed[j]]
        if not candidates:
            raise ValueError(f'No signed plane crossing found for {label} event {i}')
        j = candidates[-1]
        f = -signed[j-1] / (signed[j] - signed[j-1])
        p = pos[j-1] + f * (pos[j] - pos[j-1])
        assert abs(np.dot(p-center, n)) < 1e-10
        events.append(dict(number=len(events)+1, index=i, plane_index=j, center=center.tolist(),
                           normal_rad=angle, line_rad=float(wrap(angle+np.pi/2)),
                           crossing=p.tolist(), count_position=pos[i].tolist(),
                           crossing_time_s=float(t[j-1] + f*(t[j]-t[j-1])), count_time_s=float(t[i]),
                           crossing_bracket_s=float(t[j]-t[j-1]),
                           count_lateral_cm=float(np.dot(pos[i]-center, tangent)*100),
                           count_heading_deg=float(np.degrees(wrap(h[i]['yaw']-angle)))))
    first = events[0]['index']
    mpc_idx = [i for i,s in enumerate(h[:first]) if s['control_source'] == 2]
    mpc = [h[i] for i in mpc_idx]
    replay = json.loads((DATA / f'slam_replay_{label}.json').read_text())
    # The first corrected pose defines exactly one rigid display alignment.
    valid = [p for p in replay['poses'] if p['valid'] and 'time' in p]
    p0 = valid[0]
    angle = p0['odom_yaw'] - p0['yaw']
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    translation = np.array([p0['odom_x'], p0['odom_y']]) - rotation @ np.array([p0['x'], p0['y']])
    # One final received pose per scan sequence; no resampling of the map grid.
    unique = {p['sequence']: p for p in valid}
    replay_poses = list(unique.values())
    replay_xy = np.array([[p['x'], p['y']] for p in replay_poses]) @ rotation.T + translation
    replay_events = []
    for event in events:
        timestamp = h[event['index']]['time']
        p = min(replay_poses, key=lambda item: abs(item['time']-timestamp))
        if abs(p['time']-timestamp) > .3:
            raise ValueError(f'No nearby replay pose for gate {label}{event["number"]}')
        correction = p['yaw'] - p['odom_yaw']
        correction_rotation = np.array([[np.cos(correction), -np.sin(correction)],
                                        [np.sin(correction), np.cos(correction)]])
        projected = dict(event)
        for key in ['center', 'crossing', 'count_position']:
            local = np.array([p['x'], p['y']]) + correction_rotation @ (np.array(event[key]) - np.array([p['odom_x'], p['odom_y']]))
            projected[key] = (rotation @ local + translation).tolist()
        projected['normal_rad'] = float(wrap(event['normal_rad'] + correction + angle))
        projected['line_rad'] = float(wrap(projected['normal_rad'] + np.pi/2))
        projected['replay_time_difference_s'] = float(p['time']-timestamp)
        replay_events.append(projected)
    nominal = np.array([d['scene']['world']['start']['x'], d['scene']['world']['start']['y']])
    result = dict(label=label, path=path, d=d, h=h, t=t, pos=pos, accepted=a, move=move, events=events,
                  mpc=mpc, mpc_idx=mpc_idx, replay=replay, rotation=rotation, translation=translation,
                  replay_poses=replay_poses, replay_xy=replay_xy, replay_events=replay_events, color=color,
                  offset_cm=(pos[a]-nominal)*100, yaw_offset_deg=np.degrees(h[a]['yaw']),
                  slam_percent=100*sum(s['slam_pose_valid']>0 for s in h)/len(h),
                  tracking_mean_ms=float(np.mean([s['tracking_ms'] for s in mpc])),
                  tracking_p95_ms=float(np.percentile([s['tracking_ms'] for s in mpc], 95)),
                  speed_mean=float(np.mean([s['speed'] for s in mpc])),
                  first_mpc_time=float(t[mpc_idx[0]]))
    return result


def draw_gate(ax, event, length=.55, labels=True):
    center = np.array(event['center'])
    angle = event['normal_rad']
    normal = np.array([np.cos(angle), np.sin(angle)])
    tangent = np.array([-normal[1], normal[0]])
    ends = center + np.array([[-length/2], [length/2]]) * tangent
    ax.plot(ends[:,0], ends[:,1], color=RED, lw=2, zorder=6)
    ax.annotate('', center + normal*.13, center, arrowprops=dict(arrowstyle='->', color=RED, lw=1.3), zorder=6)
    cross, count = np.array(event['crossing']), np.array(event['count_position'])
    ax.scatter(*cross, s=55, facecolors='white', edgecolors=RED, linewidth=1.5, zorder=9)
    ax.scatter(*count, marker='D', s=29, color=RED, edgecolor='white', linewidth=.5, zorder=10)
    if labels:
        ax.annotate('G'+str(event['number']), center, xytext=(7, 8), textcoords='offset points', color=RED, weight='bold')


def map_axes(ax):
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(alpha=.5)


def grid_image(run):
    m = run['replay']['final_map']
    grid = np.full((m['height'],m['width']), 0, dtype=np.uint8)
    grid.ravel()[m['free']] = 1
    grid.ravel()[m['occupied']] = 2
    extent = [m['origin_x'],m['origin_x']+m['width']*m['resolution'],
              m['origin_y'],m['origin_y']+m['height']*m['resolution']]
    return grid, extent


def run_figures(run):
    label, color, h = run['label'], run['color'], run['h']
    fig, axes = plt.subplots(1,2, figsize=(9.0,4.8), layout='constrained')
    ax=axes[0]
    f=run['d']['frame']
    for field, c, size in [('global_free_points','#EFF2F4',7),('global_occupied_points','#56616A',9)]:
        points=np.array([[p['x'],p['y']] for p in f[field]])
        if len(points): ax.scatter(*points.T,s=size,c=c,marker='s',linewidths=0,rasterized=True)
    pos=run['pos'][run['accepted']:]
    ax.plot(*pos.T,color=color,lw=1.7,label='Navigazione')
    live=np.array([[s['slam_pose_x'],s['slam_pose_y']] for s in h[run['accepted']:] if s['slam_pose_valid']])
    ax.plot(*live.T, color=NAVY, ls='--', lw=1.1, label='Posa SLAM online')
    ax.scatter(*pos[0],marker='^',s=50,color=color,zorder=10)
    for e in run['events']: draw_gate(ax,e)
    ax.set_title('Mappa locale e gate registrati')
    ax.legend(loc='best', fontsize=7, framealpha=.9)
    map_axes(ax)
    grid, extent=grid_image(run)
    ax=axes[1]
    angle=np.arctan2(run['rotation'][1,0],run['rotation'][0,0])
    trans=Affine2D().rotate(angle).translate(*run['translation'])
    ax.imshow(grid,origin='lower',extent=extent,interpolation='nearest',cmap=ListedColormap(['#DDE3E7','white','#273A49']),vmin=0,vmax=2,transform=trans+ax.transData)
    corners=np.array([[extent[0],extent[2]],[extent[1],extent[2]],[extent[1],extent[3]],[extent[0],extent[3]]]) @ run['rotation'].T + run['translation']
    ax.set_xlim(corners[:,0].min()-.1,corners[:,0].max()+.1)
    ax.set_ylim(corners[:,1].min()-.1,corners[:,1].max()+.1)
    ax.plot(*run['replay_xy'].T,color=color,lw=1.7)
    ax.scatter(*run['replay_xy'][0],marker='^',s=50,color=color,zorder=6)
    for e in run['replay_events']:
        draw_gate(ax,e,length=.55)
    ax.set_title('Ricostruzione SLAM • replay offline')
    map_axes(ax)
    save(fig,f'run_{label}_mappe')
    # Scientific rendering of actual occupancy values, with no morphological edits.
    fig, ax=plt.subplots(figsize=(7.8,3.65),layout='constrained')
    ax.imshow(grid,origin='lower',extent=extent,interpolation='nearest',cmap=ListedColormap(['#E2E7EB','white','#172D3E']),vmin=0,vmax=2)
    ax.set_aspect('equal'); ax.set_axis_off()
    length=.5; bx=extent[0]+.10; by=extent[2]+.10
    ax.plot([bx,bx+length],[by,by],color='#111111',lw=3)
    ax.text(bx+length/2,by+.055,'0,50 m',ha='center',va='bottom',fontsize=9,bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=2))
    save(fig,f'run_{label}_slam_pulita')


def offset_figure(runs):
    fig, axes=plt.subplots(1,2,figsize=(8.8,4.0),layout='constrained',gridspec_kw={'width_ratios':[1.35,1]})
    ax=axes[0];theta=np.linspace(0,2*np.pi,250)
    ax.plot(8*np.cos(theta),8*np.sin(theta),ls='--',c='#B5BEC5',label='Tolleranza dummy: 8 cm')
    ax.scatter(0,0,marker='+',s=95,color=NAVY)
    for run in runs:
        dx,dy=run['offset_cm'];ax.annotate('',(dx,dy),(0,0),arrowprops=dict(arrowstyle='->',lw=2,color=run['color']))
        ax.scatter(dx,dy,color=run['color'],s=45)
        shift={'A':(12,-20),'B':(-35,-2),'C':(12,10)}[run['label']]
        ax.annotate(run['label'],(dx,dy),xytext=shift,textcoords='offset points',color=run['color'],weight='bold')
    ax.axhline(0,lw=.6,color='#B5BEC5');ax.axvline(0,lw=.6,color='#B5BEC5');ax.set_aspect('equal')
    ax.set(xlim=(-9,9),ylim=(-9,9),xlabel='Offset x [cm]',ylabel='Offset y [cm]',title='Posizione rispetto allo start nominale')
    ax.legend(loc='upper center',fontsize=7)
    ax=axes[1];v=[r['yaw_offset_deg'] for r in runs]
    ax.bar([r['label'] for r in runs],v,color=COLORS,width=.48);ax.axhline(0,c=GREY,lw=.8)
    for i,x in enumerate(v):ax.text(i,x+(.14 if x>=0 else -.16),number(x)+'°',ha='center',va='bottom' if x>=0 else 'top',weight='bold',color=COLORS[i])
    ax.set(ylim=(-2.1,2.65),ylabel='Offset yaw [°]',title='Orientamento iniziale');ax.grid(axis='y',alpha=.4);ax.set_axisbelow(True)
    save(fig,'01_offset_partenza')


def crossing_figure(runs):
    fig,axes=plt.subplots(2,2,figsize=(8.7,6.0),layout='constrained')
    pairs=[(r,e) for r in runs for e in r['events']]
    for ax,(run,e) in zip(axes.flat,pairs):
        i=e['index'];s=max(run['accepted'],i-42);q=min(len(run['h']),i+10)
        p=run['pos'][s:q];ax.plot(*p.T,c=run['color'],lw=2)
        draw_gate(ax,e,length=.5,labels=False)
        c=np.array(e['center']);ax.set_xlim(c[0]-.24,c[0]+.24);ax.set_ylim(c[1]-.24,c[1]+.24)
        ax.set_title(f"Run {run['label']} · G{e['number']} · normale {number(np.degrees(e['normal_rad']),1)}°")
        map_axes(ax)
    save(fig,'05_attraversamenti')


def mpc_figure(runs):
    fig,axes=plt.subplots(3,1,figsize=(8.8,5.7),layout='constrained',sharex=True)
    for ax,r in zip(axes,runs):
        end=r['events'][0]['index'];t=r['t'][:end+1];h=r['h'][:end+1]
        ax.axvspan(r['first_mpc_time'],r['events'][0]['count_time_s'],color=r['color'],alpha=.12)
        ax.plot(t,[s['planner_target_yaw_rate'] for s in h],c=r['color'],lw=1.8,label='Richiesta nominale')
        ax.plot(t,[s['body_yaw_rate_measurement'] if s['body_yaw_rate_source'] else np.nan for s in h],c=NAVY,lw=.8,alpha=.8,label='Misura angolare selezionata')
        ax.axvline(r['events'][0]['count_time_s'],color=RED,lw=1,ls='--')
        ax.axhline(0,color=GREY,lw=.5);ax.set_ylabel('ω [rad/s]');ax.set_title('Run '+r['label'],loc='left');ax.grid(alpha=.5);ax.set_xlim(0,20)
    axes[0].legend(loc='upper left',fontsize=8,ncol=2);axes[-1].set_xlabel('Tempo dal primo comando PWM non nullo [s, clock MCU]')
    save(fig,'07_mpc_tre_run')


def baseline_data():
    result=[]
    for title,name in [('Precedente','gate_trace3'),('Attuale','gate_mpc_kinematics')]:
        path=ROOT/f'documentation/corrections_20261010_lidar/{name}.log'
        lines=path.read_text().splitlines()
        rows=[r for r in csv.reader(lines) if r and r[0]=='trace']
        a=np.array([[float(x) for x in r[1:13]] for r in rows])
        # Test fixture's physical gate: start.x + 0.50 m, lateral offset -0.18 m.
        plane=a[0,1]+.50;gate_y=a[0,2]-.18
        i=next(i for i in range(1,len(a)) if a[i-1,1]<plane<=a[i,1])
        f=(plane-a[i-1,1])/(a[i,1]-a[i-1,1]);p=a[i-1]+f*(a[i]-a[i-1])
        trajectory=np.vstack([a[:i,1:3],p[1:3]])
        length=np.linalg.norm(np.diff(trajectory,axis=0),axis=1).sum()
        summary=next(l for l in lines if l.startswith('pwm_gate '))
        values=dict(re.findall(r'(\w+)=([^ ]+)',summary))
        result.append(dict(title=title,path=path,array=a,plane=plane,gate_y=gate_y,cross=p,
                           time_s=float(p[0]*.15),length_m=float(length),
                           route_efficiency=float(np.linalg.norm(trajectory[-1]-trajectory[0])/length),
                           yaw_deg=float(abs(np.degrees(p[3]))),lateral_cm=float(abs(p[2]-gate_y)*100),
                           passed=int(values['passed']),gate_steps=int(values['gate_steps']),
                           mpc_speed=float(values['gate_mean_speed'])))
    return result


def baseline_figure(baseline):
    fig,axes=plt.subplots(1,3,figsize=(9.0,3.1),layout='constrained',gridspec_kw={'width_ratios':[1.5,1,1]})
    colors=['#8B98A3','#147D92']
    for r,c in zip(baseline,colors):axes[0].plot(r['array'][:,1],r['array'][:,2],color=c,lw=2,label=r['title'])
    axes[0].plot([baseline[0]['plane']]*2,[.24,.60],c=RED,lw=2,label='Piano fisico del test')
    axes[0].scatter(baseline[0]['plane'],.42,marker='+',s=85,c=RED)
    axes[0].set_title('Stesso gate obliquo simulato');map_axes(axes[0]);axes[0].legend(fontsize=7)
    for ax,key,title,unit in [(axes[1],'time_s','Tempo al piano','s'),(axes[2],'length_m','Percorso al piano','m')]:
        vals=[r[key] for r in baseline];ax.bar(['Prec.','Attuale'],vals,color=colors,width=.55)
        for i,v in enumerate(vals):ax.text(i,v*1.025,number(v,2),ha='center',fontsize=10,weight='bold')
        ax.set_ylim(0,max(vals)*1.23);ax.set_title(title);ax.set_ylabel(unit);ax.grid(axis='y',alpha=.4);ax.set_axisbelow(True)
    save(fig,'08_efficienza_mpc')


class Report:
    def __init__(self):
        for name,weight in [('Report','normal'),('ReportBold','bold')]:
            font=font_manager.findfont(font_manager.FontProperties(family='DejaVu Sans',weight=weight))
            pdfmetrics.registerFont(TTFont(name,font))
        pdfmetrics.registerFontFamily('Report',normal='Report',bold='ReportBold',italic='Report',boldItalic='ReportBold')
        self.c=canvas.Canvas(str(OUT/'Recap_tre_prove_MPC_SLAM_20261014.pdf'),pagesize=(595.276,841.890),pageCompression=1)
        self.c.setTitle('Tre prove di navigazione autonoma | MPC, gate e SLAM | 14 ottobre 2026')
        self.c.setAuthor('Progetto Autonomous Agent')
        self.c.setSubject('Recap sperimentale delle run A, B e C; mappe SLAM offline e confronto MPC')
        self.page=0;self.y=0
        self.style=ParagraphStyle('body',fontName='Report',fontSize=9.3,leading=14,textColor=HexColor(NAVY),spaceAfter=6)

    def new_page(self,kicker,title,subtitle=''):
        if self.page:self.c.showPage()
        self.page+=1;c=self.c
        c.setFillColor(HexColor(NAVY));c.rect(0,816,596,26,fill=1,stroke=0)
        c.setFillColor(HexColor('#FFFFFF'));c.setFont('ReportBold',8);c.drawString(42,825,'AUTONOMOUS AGENT   /   REPORT SPERIMENTALE')
        c.setFillColor(HexColor(GREY));c.setFont('ReportBold',8.5);c.drawString(42,787,kicker.upper())
        c.setFillColor(HexColor(NAVY));c.setFont('ReportBold',21);c.drawString(42,756,title)
        self.y=738
        if subtitle:self.para(subtitle,size=9,color=GREY)
        c.setStrokeColor(HexColor('#DBE2E7'));c.line(42,39,553,39)
        c.setFillColor(HexColor(GREY));c.setFont('Report',7.2)
        c.drawString(42,26,'Tre run del 14 ottobre 2026 · A / B / C')
        c.drawRightString(553,26,f'{self.page:02d}')

    def para(self,text,size=9.3,color=NAVY,gap=8):
        style=ParagraphStyle('p',parent=self.style,fontSize=size,leading=size*1.5,textColor=HexColor(color))
        p=Paragraph(text,style);_,height=p.wrap(511,800)
        if self.y-height<51:raise RuntimeError(f'Paragraph overflow page {self.page}: {text[:70]} y={self.y}')
        p.drawOn(self.c,42,self.y-height);self.y-=height+gap

    def sub(self,text):
        self.y-=4;self.para('<b>'+text+'</b>',size=11,gap=8)

    def figure(self,name,height):
        from PIL import Image
        path=FIG/(name+'.png');w,h=Image.open(path).size
        width=min(511,height*w/h);actual=width*h/w
        if self.y-actual<50:raise RuntimeError(f'Figure overflow page {self.page}: {name} y={self.y}')
        self.c.drawImage(str(path),42+(511-width)/2,self.y-actual,width=width,height=actual)
        self.y-=actual+10

    def table(self,rows,widths=None,font=8):
        style=ParagraphStyle('cell',parent=self.style,fontSize=font,leading=font*1.35)
        cells=[[Paragraph(str(v),style) for v in row] for row in rows]
        table=Table(cells,colWidths=widths or [511/len(rows[0])]*len(rows[0]))
        table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),HexColor('#E9F0F4')),
                                  ('ROWBACKGROUNDS',(0,1),(-1,-1),[HexColor('#FFFFFF'),HexColor('#F5F7F9')]),
                                  ('LINEBELOW',(0,0),(-1,0),.7,HexColor('#C1CDD6')),
                                  ('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),6),
                                  ('BOTTOMPADDING',(0,0),(-1,-1),6),('LEFTPADDING',(0,0),(-1,-1),7)]))
        _,height=table.wrap(511,800)
        if self.y-height<50:raise RuntimeError(f'Table overflow page {self.page} y={self.y} height={height}')
        table.drawOn(self.c,42,self.y-height);self.y-=height+12


def build_pdf(runs,baseline):
    p=Report()
    p.new_page('01 / Quadro sperimentale','Tre prove di navigazione autonoma','Offset iniziale, ricostruzione SLAM e attraversamento dei gate · 14 ottobre 2026')
    p.para('Le tre acquisizioni esplorano un ambiente indoor mediante LiDAR, localizzazione encoder/IMU e inseguimento MPC di riferimenti a clotoide. Il robot misura 25 × 15 cm; il LiDAR è un RPLidar A1 con portata operativa di 1,20 m. Lo start nominale nel riferimento del planner è (0,220; 0,600) m, con yaw 0°.')
    rows=[['<b>Run</b>','<b>Snapshot</b>','<b>Campioni</b>','<b>Gate registrati</b>','<b>Pose SLAM valide</b>']]
    for r in runs:rows.append([r['label'],{'A':'00:19:26','B':'00:22:33','C':'00:25:48'}[r['label']],len(r['h']),len(r['events']),number(r['slam_percent'])+'%'])
    p.table(rows,[42,99,92,111,167])
    p.sub('Offset stimato alla partenza')
    p.figure('01_offset_partenza',270)
    rows=[['<b>Run</b>','<b>Δx [cm]</b>','<b>Δy [cm]</b>','<b>Modulo [cm]</b>','<b>Δyaw [°]</b>','<b>Confidenza</b>']]
    for r in runs:rows.append([r['label'],number(r['offset_cm'][0]),number(r['offset_cm'][1]),number(np.linalg.norm(r['offset_cm'])),number(r['yaw_offset_deg']),number(r['h'][r['accepted']]['start_matching_confidence'],3)])
    p.table(rows,[40,86,86,112,85,102])
    p.para('Gli offset sono ricavati dalla posa accettata dal matcher rispetto allo start nominale. La tolleranza della serie dummy è 8 cm / 8°. In tutte le run i PWM e gli incrementi encoder precedenti all’accettazione sono nulli.',size=8.5)

    descriptions={
        'A':'Il primo inseguimento si sviluppa verso un gate quasi frontale. Il piano registrato ha normale a −10,0°. Dopo il passaggio il percorso prosegue nella zona adiacente al varco.',
        'B':'La prima apertura selezionata si trova in una zona simile a quella della A. Al conteggio del passaggio lo yaw stimato forma un angolo di +26,0° rispetto alla normale del gate; la differenza è visibile nel dettaglio geometrico.',
        'C':'La fase di ricerca precede un avvicinamento orientato prevalentemente lungo +y. La run registra due passaggi, su piani con normali differenti; il primo target si distingue geometricamente da quelli delle run A e B.'}
    for n,r in enumerate(runs,2):
        p.new_page(f'{n:02d} / Mappe e traiettorie',f"Run {r['label']} · ricostruzione dell’ambiente",{'A':'Snapshot 00:19:26','B':'Snapshot 00:22:33','C':'Snapshot 00:25:48'}[r['label']])
        p.para(descriptions[r['label']])
        p.figure(f"run_{r['label']}_mappe",285)
        p.para('Sinistra: griglia locale esportata, navigazione e posa SLAM online tratteggiata. Le linee rosse sono i piani registrati dal planner; le frecce indicano le normali. Destra: mappa e percorso del replay SLAM, con i gate trasferiti mediante la trasformazione rigida locale all’evento. Cerchi: intersezioni; rombi: conteggi.',size=8.1,gap=6)
        p.sub('Mappa SLAM effettiva · vista pulita')
        p.figure(f"run_{r['label']}_slam_pulita",185)
        m=r['replay']['final_map']
        p.para(f"Output di <b>slam_toolbox</b> ricalcolato dalle scansioni archiviate: {r['replay']['scan_count']} scansioni inviate, {m['graph_nodes']} nodi nel grafo finale. Risoluzione {number(m['resolution']*100,1)} cm/cella. Scuro: occupato; bianco: libero; grigio: non osservato. La pulizia riguarda la presentazione, senza cancellazione di celle o regolarizzazione delle pareti.",size=8.1)

    p.new_page('05 / Geometria dei passaggi','Dove vengono attraversati i gate','Coordinate nel riferimento di navigazione del planner; angoli misurati da +x in senso antiorario.')
    p.figure('05_attraversamenti',325)
    p.para('<b>Cerchio vuoto:</b> intersezione interpolata della traiettoria stimata con il piano. <b>Rombo pieno:</b> campione in cui aumenta il contatore dei gate. Il segmento rosso rappresenta il piano: la sua lunghezza è illustrativa e non misura la larghezza del varco.',size=8.5)
    rows=[['<b>Gate</b>','<b>Intersezione x; y [m]</b>','<b>Conteggio x; y [m]</b>','<b>Normale / linea [°]</b>','<b>t conteggio [s]</b>']]
    for r in runs:
        for e in r['events']:
            rows.append([r['label']+str(e['number']),'; '.join(number(x,3) for x in e['crossing']),'; '.join(number(x,3) for x in e['count_position']),number(np.degrees(e['normal_rad']),1)+' / '+number(np.degrees(e['line_rad']),1),number(e['count_time_s'],3)])
    p.table(rows,[40,131,131,113,96],font=7.7)
    p.para('Il piano è ricostruito con centro e heading del gate bloccato nell’ultimo campione precedente al conteggio. L’intersezione usa il cambio di segno della distanza lungo la normale e un’interpolazione lineare fra due pose. Le tre cifre decimali descrivono i dati esportati, non una precisione fisica millimetrica.',size=8.4)
    p.para('I tempi usano il timestamp encoder MCU, con origine al primo PWM non nullo. Il conteggio avviene dopo il passaggio del centro stimato; l’evento non identifica da solo il superamento dell’intero ingombro del robot.',size=8.4)

    p.new_page('06 / Controllo predittivo','Come opera l’MPC in queste prove','Dalla geometria del gate al comando delle due ruote.')
    p.sub('1. Il gate definisce una geometria da seguire')
    p.para('Il planner ricava centro e normale del varco dai ritorni LiDAR. La normale orienta il riferimento attraverso l’apertura. Una clotoide campionata fornisce posizione, orientamento, curvatura e velocità desiderati; la geometria del corpo viene verificata lungo il percorso.')
    p.sub('2. L’MPC valuta un orizzonte breve')
    p.para('Per la car hardware in esplorazione aperta l’orizzonte è di <b>8 passi da 0,08 s</b>, pari a <b>0,64 s</b>. Il preview geometrico è 10 cm e il lookahead minimo 5,5 cm. A ogni aggiornamento vengono valutate <b>25 coppie</b> candidate di accelerazione e velocità di sterzo virtuale, mantenute costanti nella singola previsione.')
    p.table([['<b>Termine del costo</b>','<b>Peso</b>','<b>Ruolo</b>'],
             ['Errore trasversale / angolare','32 / 22','Allineare il corpo al riferimento'],
             ['Errore di velocità / sterzo','0,65 / 0,30','Seguire il profilo e la curvatura'],
             ['Accelerazione / velocità di sterzo','0,14 / 0,06','Penalizzare azioni intense'],
             ['Fattore terminale','12','Pesare maggiormente l’ultimo errore previsto']], [204,86,221])
    p.para('Il costo somma i quadrati degli errori lungo l’orizzonte e le penalità sui comandi. L’algoritmo sceglie la coppia con costo minimo; al campione seguente aggiorna stato, riferimento e previsione. Si tratta di una ricerca discreta su azioni candidate, non della soluzione di un programma quadratico generale.',size=8.8)
    p.sub('3. Il modello segue la cinematica dei motori differenziali')
    p.para('Nella previsione attuale la velocità del corpo è lungo il suo yaw: <b>ẋ = v cos ψ; ẏ = v sin ψ; ψ̇ = v tan δ / L</b>, con passo L = 0,18 m. Lo sterzo δ è virtuale. Il riferimento di curvatura viene convertito in velocità angolare e comandi differenziali alle ruote; il feedback inerziale e la regolazione PWM intervengono a valle.')
    p.para('Il precedente modello hardware includeva β = atan[(l<sub>r</sub>/L) tan δ], con l<sub>r</sub> = 0,09 m, prevedendo traslazione lungo ψ + β. Il modello attuale pone questo offset a zero nella sola previsione, coerentemente con i due motori differenziali. Pesi, clotoide e ingombro conservano il proprio ruolo.',size=8.8)
    p.sub('4. Applicazione alle run A, B e C')
    p.para('L’MPC entra in azione quando è disponibile un gate con riferimento valido. Prima dell’aggancio opera la ricerca dell’apertura: per questo la diversa direzione iniziale della C precede il suo inseguimento MPC. Lo SLAM online è acquisito in osservazione; lo stato utilizzato per il controllo proviene dalla navigazione encoder/IMU.',size=8.8)

    p.new_page('07 / MPC nelle acquisizioni','L’inseguimento nelle tre run','Analisi fino al primo gate registrato in ciascuna acquisizione.')
    p.figure('07_mpc_tre_run',345)
    p.para('Lo sfondo colorato identifica la finestra fra primo aggancio MPC e conteggio del gate. La linea colorata è la richiesta angolare nominale del planner; quella scura è la misura angolare selezionata dal controllo. La linea verticale tratteggiata individua il passaggio registrato.',size=8.4)
    rows=[['<b>Indicatore</b>','<b>A</b>','<b>B</b>','<b>C</b>']]
    rows.extend([
        ['Aggancio MPC dal primo comando [s]']+[number(r['first_mpc_time'],3) for r in runs],
        ['Primo gate dal primo comando [s]']+[number(r['events'][0]['count_time_s'],3) for r in runs],
        ['Campioni MPC con riferimento']+[f"{sum(s['planner_has_reference']>0 for s in r['mpc'])}/{len(r['mpc'])}" for r in runs],
        ['Velocità media encoder/IMU in MPC [m/s]']+[number(r['speed_mean'],3) for r in runs],
        ['Massimo errore trasversale interno [cm]']+[number(max(s['tracker_cross_track'] for s in r['mpc'])*100) for r in runs],
        ['Tracking: tempo medio / p95 [ms]']+[number(r['tracking_mean_ms'],3)+' / '+number(r['tracking_p95_ms'],3) for r in runs],
    ])
    p.table(rows,[235,92,92,92],font=8)
    p.para('La durata della ricerca spiega parte della differenza fra i tempi totali. L’errore del tracker è relativo al riferimento locale, che può essere rigenerato. Il tempo di tracking comprende il ramo di controllo misurato dal runner: non è un benchmark isolato del solo solver.',size=8.3)

    p.new_page('08 / Confronto con il precedente MPC','Efficienza e geometria di esecuzione','Confronto controllato su log simulati archiviati; collegamento con il modello utilizzato nelle tre run.')
    p.para('Il riferimento “precedente” è l’MPC hardware immediatamente prima dell’aggiornamento cinematico del 10 ottobre. I due log descrivono lo stesso test: partenza obliqua a +0,35 rad, gate spostato di −0,18 m e ciclo a 150 ms. Il confronto usa la posa del modello fisico simulato fino al piano x = 0,720 m.')
    p.figure('08_efficienza_mpc',235)
    b,a=baseline
    rows=[['<b>Misura sullo stesso test</b>','<b>Precedente</b>','<b>Attuale</b>'],
          ['Tempo fino al piano del gate [s]',number(b['time_s'],3),number(a['time_s'],3)],
          ['Percorso fino al piano [m]',number(b['length_m'],3),number(a['length_m'],3)],
          ['Rapporto distanza diretta / percorso',number(b['route_efficiency'],3),number(a['route_efficiency'],3)],
          ['Angolo del corpo rispetto alla normale [°]',number(b['yaw_deg'],2),number(a['yaw_deg'],2)],
          ['Distanza laterale dal centro del gate [cm]',number(b['lateral_cm'],2),number(a['lateral_cm'],2)],
          ['Gate registrati dal planner a fine test',b['passed'],a['passed']],
          ['Valutazioni previste per chiamata MPC','25 × 8 = 200','25 × 8 = 200']]
    p.table(rows,[281,115,115],font=8.1)
    time_gain=(1-a['time_s']/b['time_s'])*100
    length_gain=(1-a['length_m']/b['length_m'])*100
    p.para(f"Nel test il modello attuale raggiunge il piano in un tempo inferiore del <b>{number(time_gain,1)}%</b> e con un percorso inferiore del <b>{number(length_gain,1)}%</b>. L’allineamento angolare migliora; lo scarto laterale dal centro cresce. Il carico teorico della ricerca MPC resta pari a 200 passi predetti per chiamata.",size=8.8)
    p.para('Questi numeri confrontano una singola manovra simulata e descrivono tempo, percorso e geometria. Non misurano consumo energetico né un guadagno percentuale sul robot. Le tre acquisizioni del 14 ottobre usano tutte il modello attuale; il collegamento sperimentale è dato dai segnali e dai tempi di tracking della pagina precedente.',size=8.5)

    p.new_page('09 / Note di lettura e provenienza','Metodo e fonti dei dati','Dettagli necessari per interpretare e riprodurre le figure.')
    p.sub('Riferimenti spaziali e temporali')
    p.para('Le coordinate tabulate dei gate sono nel riferimento del planner. Le viste SLAM usano un allineamento rigido alla prima posa valida del replay. I gate vengono trasferiti con la correzione posa SLAM/odometria del campione dell’evento: sono geometrie del planner, non varchi rilevati autonomamente dallo SLAM. Nelle viste pulite si conserva la griglia nativa, senza adattare le pareti al disegno nominale dell’arena.')
    p.para('Il tempo delle run parte dal primo comando PWM non nullo ed è calcolato sul clock encoder MCU. La frequenza dei campioni e la latenza delle risposte pongono un limite alla risoluzione degli eventi. Il punto di intersezione è una stima interpolata; il punto del conteggio è una posa effettivamente contenuta nell’export.')
    p.sub('Mappe SLAM')
    p.para('La griglia originale mostrata dalla GUI è identificata negli export come persistent_local_grid. Le mappe SLAM qui presentate provengono da un nuovo replay, separato per run, delle scansioni a partire dall’accettazione della posa iniziale. Il backend è ROS 2 slam_toolbox con ottimizzazione Ceres, scan matching e ricerca di loop; la risoluzione è 0,025 m.')
    p.para('Il replay conserva l’ordine e gli intervalli del tempo interno esportato dopo l’accettazione. Gli angoli e l’origine del sensore vengono ricavati dal CSV LiDAR. Non è una copia della mappa storica online: reset, scheduling e associazioni del grafo possono produrre un risultato differente. Le traiettorie del replay sono pose restituite progressivamente dal backend, non un’esportazione finale di tutte le pose del grafo riottimizzato.',size=8.8)
    p.para('Nel report le stime di navigazione, SLAM online e SLAM offline mantengono etichette distinte. Non è presente una misura esterna della posa nelle tre acquisizioni. La disponibilità SLAM indica quanti campioni hanno una posa valida, non la sua accuratezza metrica.',size=8.8)
    p.sub('Materiale sorgente')
    p.para('<b>Run A / B / C:</b> snapshot JSON e CSV LiDAR con suffissi 20261014_001926_149, 20261014_002233_702 e 20261014_002548_405, nella cartella reports. Profilo car_measured_2026_07, versione 1.8.0; firmware dichiarato 1.5.',size=8.5)
    p.para('<b>Confronto precedente/attuale:</b> gate_trace3.log e gate_mpc_kinematics.log in documentation/corrections_20261010_lidar; descrizione in LiDAR_Physical_Calibration_Report_20261010.md. I tempi al piano sono ricalcolati interpolando le pose del modello simulato, con periodo 0,15 s.',size=8.5)
    p.para('<b>Definizione del controllo:</b> mpc_path_follower.cpp e .h; hardware_planner/parts/lifecycle.inc e control.inc. Le quantità riportate derivano dalla configurazione hardware e dalle chiamate effettive al follower.',size=8.5)
    p.para('<b>Allegati riproducibili:</b> figure PNG/PDF, metriche JSON, coordinate dei gate CSV, output dei tre replay e manifest SHA-256 sono salvati insieme al documento. Gli script sono scripts/replay_dummy_report_slam.py e scripts/build_dummy_colleague_report.py.',size=8.5)
    p.c.save()


def main():
    FIG.mkdir(parents=True,exist_ok=True);DATA.mkdir(parents=True,exist_ok=True)
    runs=[load_run(label,suffix,color) for label,suffix,color in zip('ABC',SUFFIXES,COLORS)]
    for run in runs:run_figures(run)
    offset_figure(runs);crossing_figure(runs);mpc_figure(runs)
    baseline=baseline_data();baseline_figure(baseline)
    metrics=[]
    for r in runs:
        metrics.append({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in r.items() if k in ['label','events','replay_events','offset_cm','yaw_offset_deg','slam_percent','tracking_mean_ms','tracking_p95_ms','speed_mean','first_mpc_time']})
    bm=[{k:v for k,v in r.items() if k not in ['array','cross','path']} for r in baseline]
    (DATA/'metriche_report.json').write_text(json.dumps(dict(runs=metrics,mpc_controlled_comparison=bm),indent=2))
    with (DATA/'attraversamenti_gate.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['run','gate','plane_x_m','plane_y_m','normal_deg','line_deg','crossing_x_m','crossing_y_m','count_x_m','count_y_m','crossing_time_s','count_time_s','bracket_s','lateral_at_count_cm','heading_at_count_deg'])
        for r in runs:
            for e in r['events']:writer.writerow([r['label'],e['number'],*e['center'],np.degrees(e['normal_rad']),np.degrees(e['line_rad']),*e['crossing'],*e['count_position'],e['crossing_time_s'],e['count_time_s'],e['crossing_bracket_s'],e['count_lateral_cm'],e['count_heading_deg']])
    build_pdf(runs,baseline)
    inputs=[]
    for r in runs:inputs += [r['path'],r['path'].with_name(r['path'].stem+'_lidar_reconstruction.csv')]
    inputs += [r['path'] for r in baseline]
    inputs += [ROOT/'tools/slam_toolbox_bridge/mapper_params.yaml',Path(__file__),ROOT/'scripts/replay_dummy_report_slam.py']
    inputs += [DATA/'replay_mapper_params.yaml',DATA/'replay_bridge_node.py']
    manifest=dict(date='2026-10-14',time_zone='Europe/Rome',
                  backend_image='sha256:308f16a64991fbc1040f4bd8d4d9cd070fe8e9cf5682e816cc92461cc9a542b5',
                  source_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs})
    (DATA/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(dict(pdf=str(OUT/'Recap_tre_prove_MPC_SLAM_20261014.pdf'),runs=metrics,baseline=bm),indent=2))


if __name__=='__main__':main()
