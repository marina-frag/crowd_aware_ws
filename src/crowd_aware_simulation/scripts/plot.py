#!/usr/bin/env python3
"""Animated r(phi,t) and S(phi,t) from a recorded bag; Space=play/pause."""
import argparse
import bisect
import json
import math


def plot_bag_main():
    from dwal_bag import events, locate, sample_geometry

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


#!/usr/bin/env python3
"""Live r(phi,t)/S(phi,t) from ROS topics. Independent of rosbag recording."""
import argparse
import json
import math
from pathlib import Path
import time

# Same eight RGB entries as upstream setColor(id % 8).
PALETTE=[(1,0,0),(0,1,0),(0,0,1),(1,1,0),(0,1,1),(1,0,1),(.5,.5,.2),(.8,.1,.4)]


def classify(rows,membership,codes):
    result=[]
    for row in rows:
        cid=None
        if membership is not None:
            cid=next((v for k,v in membership.items() if math.isclose(k,row['curvature'],rel_tol=1e-9,abs_tol=1e-10)),None)
        code=codes[row['index']] if row['index']<len(codes) else None
        if code is not None and code>=0:cid=code
        # Distinct rejection markers avoid treating short admissible paths as unsafe.
        reason={-1:'filter/invalid',-2:'braking',-3:'slice',-4:'min span'}.get(code)
        label=f'cluster {cid}' if cid is not None else (reason or 'unassigned')
        color=PALETTE[cid%len(PALETTE)] if cid is not None else ('black' if code==-2 else '0.6')
        result.append((row,label,color,'o' if cid is not None else 'x'))
    return result


def plot_live_main():
    from dwal_bag import sample_geometry

    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--active-run',type=Path,default=Path('/tmp/dwal_active_run.json'))
    p.add_argument('--group',choices=['far','near'],default='far')
    p.add_argument('--refresh-hz',type=float,default=10.)
    a=p.parse_args()
    if not math.isfinite(a.refresh_hz) or not 0<a.refresh_hz<=30:p.error('refresh-hz must be in (0,30]')
    active=json.loads(a.active_run.read_text());cfg=json.loads((Path(active['run_directory'])/'evaluation.json').read_text())
    if a.group=='near' and active['controller']=='dynamic_dwal':p.error('Dynamic mode publishes only the far group')
    topics=cfg['topics'];group_topic=topics['clusters_'+a.group];diag_topic=topics[a.group+'_diagnostics']
    import matplotlib
    matplotlib.use('TkAgg',force=True)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from dwal_planner.msg import SampledCluster,ClusterGroup
    from std_msgs.msg import Float64MultiArray,String
    rclpy.init();node=Node('dwal_live_plot',parameter_overrides=[Parameter('use_sim_time',value=True)])
    state={'rows':[],'sample_at':None,'sample_time':None,'members':None,'group_at':None,
           'codes':[],'diag_at':None,'requirement':None,'paused':False,'version':0}
    def samples(msg):
        state.update(rows=sample_geometry(msg,cfg['reference_radius_m']),sample_at=time.monotonic(),
                     sample_time=node.get_clock().now().nanoseconds*1e-9,version=state['version']+1)
    def clusters(msg):
        state.update(members={float(p.curvature):int(c.id) for c in msg.clusters for p in c.paths},group_at=time.monotonic())
    def diagnostics(msg):
        try:d=json.loads(msg.data)
        except (ValueError,AttributeError):return
        state.update(codes=d.get('codes',[]),diag_at=time.monotonic())
    def requirement(msg):
        if len(msg.data)==3:state['requirement']=list(msg.data)
    subscriptions=[
        node.create_subscription(SampledCluster,topics['sampled_paths'],samples,qos_profile_sensor_data),
        node.create_subscription(ClusterGroup,group_topic,clusters,qos_profile_sensor_data),
        node.create_subscription(String,diag_topic,diagnostics,qos_profile_sensor_data),
        node.create_subscription(Float64MultiArray,topics['braking_requirement'],requirement,qos_profile_sensor_data)]
    fig,axes=plt.subplots(2,1,sharex=True);fig.subplots_adjust(bottom=.15)
    pause=Button(fig.add_axes([.81,.025,.15,.05]),'Pause view')
    def toggle(_=None):state['paused']=not state['paused'];pause.label.set_text('Resume view' if state['paused'] else 'Pause view')
    pause.on_clicked(toggle);fig.canvas.mpl_connect('key_press_event',lambda e:toggle() if e.key==' ' else None)
    def tick(_):
        # All callbacks execute in the GUI thread; no cross-thread mutable ROS state.
        budget=time.monotonic()+.008
        for _ in range(40):
            rclpy.spin_once(node,timeout_sec=0.)
            if time.monotonic()>budget:break
        if state['paused']:return
        now=time.monotonic();sim=node.get_clock().now().nanoseconds*1e-9
        fresh=state['sample_at'] is not None and now-state['sample_at']<=cfg['odom_timeout_s']
        group_fresh=state['group_at'] is not None and now-state['group_at']<=cfg['odom_timeout_s']
        diagnostic_fresh=state['diag_at'] is not None and now-state['diag_at']<=cfg['odom_timeout_s']
        membership=state['members'] if group_fresh and fresh else None
        codes=state['codes'] if diagnostic_fresh and fresh and len(state['codes'])==len(state['rows']) else []
        # ROS messages have no shared frame sequence: freshness is shown, not exact synchronization.
        rows=state['rows'] if fresh else []
        grouped={}
        for row,label,color,marker in classify(rows,membership,codes):
            grouped.setdefault(label,{'rows':[],'color':color,'marker':marker})['rows'].append(row)
        for ax in axes:ax.clear();ax.grid(alpha=.25)
        ordered=sorted(rows,key=lambda r:r['phi'])
        for ax,key in zip(axes,('r','length')):
            ax.plot([math.degrees(r['phi']) for r in ordered],[r[key] for r in ordered],color='0.8',lw=.8)
            for label,group in grouped.items():
                ax.scatter([math.degrees(r['phi']) for r in group['rows']],[r[key] for r in group['rows']],
                           color=group['color'],marker=group['marker'],s=18,label=label)
            ax.set_xlim(-95,95);ax.set_ylim(bottom=0)
        req=state['requirement'];required=None
        if req and all(math.isfinite(x) for x in req) and req[1]>0 and -.1<=sim-req[2]<=req[1]:required=req[0]
        if required is not None:axes[1].axhline(required,color='black',ls='--',label=f'S required={required:.3f} m')
        axes[0].set_ylabel('Endpoint r [m]');axes[1].set_ylabel('Free path S [m]');axes[1].set_xlabel('phi at common reference radius [degrees]')
        axes[0].set_title(f'LIVE | {active["variant"]}/{active["semantics"]} | {a.group} | sim t={sim:.2f}s | '+('fresh data (receipt-aligned)' if fresh else 'WAITING / STALE samples'))
        if grouped:axes[0].legend(loc='best',fontsize='small')
        if required is not None:axes[1].legend(loc='best',fontsize='small')
        fig.canvas.draw_idle()
    animation=FuncAnimation(fig,tick,interval=1000/a.refresh_hz,cache_frame_data=False)
    print('Live subscriptions active. Start/stop the recorder independently. Space pauses only the view.',flush=True)
    try:plt.show()
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

#!/usr/bin/env python3
"""Exercise compiled Matplotlib extensions, actual rendering, and optional X11 GUI."""
import argparse
import io


def check_main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gui',action='store_true');a=p.parse_args()
    import numpy as np
    import matplotlib
    matplotlib.use('TkAgg' if a.gui else 'Agg',force=True)
    import matplotlib.pyplot as plt
    import tkinter
    tkinter.Tcl()  # Does not require DISPLAY; verifies the installed Tk bindings.
    print(f'NumPy {np.__version__}: {np.__file__}',flush=True)
    print(f'Matplotlib {matplotlib.__version__}: {matplotlib.__file__}',flush=True)
    figure,axis=plt.subplots()
    axis.plot(np.linspace(0,1,10),np.linspace(0,1,10)**2)
    axis.set_title('DWAL plotting compatibility check')
    figure.canvas.draw()  # Loads/executes the native rendering extensions.
    buffer=io.BytesIO();figure.savefig(buffer,format='png')
    if not buffer.getvalue().startswith(b'\x89PNG\r\n\x1a\n'):raise RuntimeError('PNG rendering failed')
    print('PASS: actual figure rendered to PNG; Tk bindings available.',flush=True)
    if a.gui:plt.show()
    plt.close(figure)

def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command')
    subparsers.add_parser('live')
    subparsers.add_parser('check-backend').add_argument('--gui', action='store_true')
    args, remaining = parser.parse_known_args()
    import sys
    if args.command == 'live':
        sys.argv = [sys.argv[0], *remaining]
        plot_live_main()
    elif args.command == 'check-backend':
        sys.argv = [sys.argv[0], *remaining]
        if args.gui:
            sys.argv.append('--gui')
        check_main()
    else:
        sys.argv = [sys.argv[0], *(([args.command] if args.command else []) + remaining)]
        plot_bag_main()

if __name__ == '__main__':
    main()
