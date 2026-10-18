#!/usr/bin/env python3
"""Animated r(phi,t) and S(phi,t) from a recorded bag; Space=play/pause."""
import argparse
import bisect
import json
import math
from dwal_bag import events, locate, sample_geometry


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--rate',type=float,default=1.)
    a=p.parse_args()
    if not math.isfinite(a.rate) or a.rate<=0:p.error('rate must be positive')
    # Fail early on an incompatible binary stack, before loading the whole bag.
    import matplotlib
    matplotlib.use('TkAgg',force=True)
    import matplotlib.pyplot as plt
    _,cfg=locate(a.bag);t=cfg['topics'];frames=[];groups=[];requirements=[];diagnostics=[]
    needed={t[k] for k in ('sampled_paths','clusters_far','braking_requirement','far_diagnostics')}
    for now,topic,msg in events(a.bag,needed):
        if topic==t['sampled_paths']:frames.append((now,sample_geometry(msg,cfg['reference_radius_m'])))
        elif topic==t['clusters_far']:groups.append((now,{float(p.curvature):int(c.id) for c in msg.clusters for p in c.paths}))
        elif topic==t['braking_requirement']:
            if len(msg.data)==3:requirements.append((now,list(msg.data)))
        else:
            try:diagnostics.append((now,json.loads(msg.data)))
            except (ValueError,AttributeError):pass
    if not frames:p.error('No sampled_paths in bag')
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider,Button
    fig,axes=plt.subplots(2,1,sharex=True);fig.subplots_adjust(bottom=.22)
    slider=Slider(fig.add_axes([.15,.08,.65,.035]),'Time [s]',0,max(frames[-1][0]-frames[0][0],1e-6),valinit=0)
    button=Button(fig.add_axes([.82,.07,.13,.055]),'Pause')
    state={'play':True};base=frames[0][0];times=[x[0] for x in frames]
    def latest(rows,now,max_age):
        i=bisect.bisect_right([x[0] for x in rows],now)-1
        return rows[i][1] if i>=0 and now-rows[i][0]<=max_age else None
    def draw(relative):
        now=base+relative;i=max(0,bisect.bisect_right(times,now)-1);sample_time,data=frames[i]
        # Select output after this input and before the next input, avoiding previous-frame ID labels.
        stop=times[i+1] if i+1<len(times) else sample_time+cfg['odom_timeout_s']
        candidates=[x for x in groups if sample_time<=x[0]<stop]
        membership=candidates[-1][1] if candidates else None
        ds=[x for x in diagnostics if sample_time<=x[0]<stop]
        codes=ds[-1][1].get('codes',[]) if ds else []
        req=latest(requirements,sample_time,cfg['odom_timeout_s'])
        required=None
        if req and len(req)==3 and math.isfinite(req[0]) and -.1<=sample_time-req[2]<=req[1]:required=req[0]
        for ax in axes:ax.clear();ax.grid(alpha=.25)
        colors=plt.get_cmap('tab10')
        ordered=sorted(data,key=lambda row:row['phi'])
        for ax,key in zip(axes,('r','length')):
            ax.plot([math.degrees(row['phi']) for row in ordered], [row[key] for row in ordered], color='0.75', lw=.8, zorder=0)
        for row in data:
            k=row['curvature'];cid=None if membership is None else membership.get(k)
            if cid is None and membership is not None:
                match=next((v for q,v in membership.items() if math.isclose(q,k,rel_tol=1e-9,abs_tol=1e-10)),None);cid=match
            color='0.65' if cid is None else colors(cid%10)
            marker='x' if cid is None else 'o'
            if row['index']<len(codes) and codes[row['index']]==-2:color='red'
            for ax,key in zip(axes,('r','length')):ax.scatter(math.degrees(row['phi']),row[key],c=[color],marker=marker,s=20)
        if required is not None:axes[1].axhline(required,color='red',ls='--',label=f'S required = {required:.3f} m');axes[1].legend()
        axes[0].set_ylabel('Endpoint r [m]');axes[1].set_ylabel('Free path S [m]');axes[1].set_xlabel('phi at common reference radius [degrees]')
        axes[0].set_title(f'{cfg.get("variant")} / {cfg.get("semantics")} | t={sample_time-base:.2f}s | '+('clusters unavailable' if membership is None else 'colors = cluster IDs; x = unassigned'))
        fig.canvas.draw_idle()
    def toggle(_=None):state['play']=not state['play'];button.label.set_text('Pause' if state['play'] else 'Play')
    button.on_clicked(toggle);slider.on_changed(draw)
    fig.canvas.mpl_connect('key_press_event',lambda e:toggle() if e.key==' ' else None)
    def tick(_):
        if state['play']:
            value=min(slider.val+.05*a.rate,slider.valmax);slider.set_val(value)
            if value>=slider.valmax:state['play']=False;button.label.set_text('Play')
    animation=FuncAnimation(fig,tick,interval=50,cache_frame_data=False)
    draw(0);plt.show()


if __name__=='__main__':main()
