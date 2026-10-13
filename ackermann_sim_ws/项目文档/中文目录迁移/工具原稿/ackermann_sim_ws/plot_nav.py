import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
p=Path('artifacts/M3_导航/验收成果/仿真_简单场地导航');r=json.loads((p/'results.json').read_text());trajectory=r.get('trajectory') or json.loads((p/r['trajectory_file']).read_text());tr=np.array(trajectory['truth'])
fig,ax=plt.subplots(figsize=(12,6));ax.plot(tr[:,2],tr[:,3],color='green',lw=2,label='Actual Gazebo model trajectory (TEST truth)')
for i,plan in enumerate(trajectory['plans']):
 a=np.array(plan);ax.plot(a[:,0],a[:,1],'--',color='tomato',alpha=.65,label='Smac Hybrid-A* plans' if i==0 else None)
for b in r['scene']['obstacles']:
 ax.add_patch(Rectangle((b['x']-b['size_x']/2,b['y']-b['size_y']/2),b['size_x'],b['size_y'],color='gray'));ax.annotate('KNOWN fixed obstacle',(b['x'],b['y']),xytext=(b['x']-1,b['y']-2))
for g in r['goals']:
 a=g['goal'];x=g['final_truth_pose'];ax.scatter(a['x'],a['y'],marker='x',s=80,color='blue');ax.annotate(f"{a['name']}: error {g['endpoint_error_m']:.3f}m",(a['x'],a['y']),xytext=(a['x']-1,a['y']-.7));ax.arrow(x['x'],x['y'],.5*math.cos(x['yaw']),.5*math.sin(x['yaw']),width=.025,color='darkgreen')
ax.set_aspect('equal');ax.set_xlim(-1,21);ax.set_ylim(-2,7);ax.set_xlabel('odom x [m]');ax.set_ylabel('odom y [m]');ax.grid();ax.legend(loc='upper left');ax.set_title('Preliminary M3 simulation: forward-only navigation, no real localization');fig.tight_layout();fig.savefig(p/'navigation_trajectory.png',dpi=150)
