#!/usr/bin/env python3
"""Live r(phi,t)/S(phi,t) from ROS topics. Independent of rosbag recording."""
import argparse
import json
import math
from pathlib import Path
import time
from dwal_bag import sample_geometry

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


def main():
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

if __name__=='__main__':main()
