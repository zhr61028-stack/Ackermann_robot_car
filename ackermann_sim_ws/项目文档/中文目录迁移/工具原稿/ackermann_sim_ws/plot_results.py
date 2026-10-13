import json,math,statistics
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
p=Path(__file__).resolve().parents[1]/'artifacts/M1_底盘与控制/过程记录/仿真_基础车辆控制旧版本'
r=json.loads((p/'results.json').read_text())
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(p/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
truth=[];odom=[];status=[];offset=[]
while reader.has_next():
 topic,data,t=reader.read_next()
 if topic=='/odom':
  m=deserialize_message(data,Odometry);odom.append((m.pose.pose.position.x,m.pose.pose.position.y))
 elif topic=='/ground_truth/poses':
  for tr in deserialize_message(data,TFMessage).transforms:
   if tr.child_frame_id=='ackermann':truth.append((tr.transform.translation.x,tr.transform.translation.y,tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,t/1e9))
 elif topic=='/sim/watchdog':
  x=json.loads(deserialize_message(data,String).data);status.append(x);offset.append(t/1e9-x['monotonic_s'])
truth=np.array(truth);odom=np.array(odom)
fig,ax=plt.subplots(figsize=(7,6));ax.plot(truth[:,0],truth[:,1],color='#13945c',lw=2,label='Gazebo physical pose');ax.plot(odom[:,0],odom[:,1],color='#2475c7',ls='--',lw=1.5,label='Native wheel odometry');ax.scatter(*truth[0,:2],c='black',s=30,label='Start');ax.scatter(*truth[-1,:2],c='#d15b20',s=35,label='Stop');ax.set_aspect('equal');ax.grid(alpha=.3);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.set_title('Ackermann simulation trajectory - assumed geometry');ax.legend();fig.tight_layout();fig.savefig(p/'trajectory.png',dpi=180);plt.close(fig)
last=r['watchdog']['last_rx_monotonic_s'];align=statistics.median(offset);dt=np.diff(truth[:,2]);valid=dt>0;speed=np.hypot(np.diff(truth[:,0]),np.diff(truth[:,1]))[valid]/dt[valid];times=truth[1:,3][valid]-align-last
fig,ax=plt.subplots(figsize=(8,4));ax.plot(times,speed,c='#13945c',label='Physical model speed');ax.step([x['monotonic_s']-last for x in status],[x['safe_speed'] for x in status],where='post',c='#2475c7',label='Guard output speed');ax.axvline(r['watchdog']['trigger_latency_wall_s'],c='#bd7421',ls='--',label='Timeout sends zero');ax.axvline(r['watchdog']['physical_stop_latency_wall_s'],c='#b62f46',ls=':',label='Physical speed below threshold');ax.axhline(.02,c='gray',lw=.8);ax.set_xlim(-.2,1);ax.set_ylim(-.025,.65);ax.set_xlabel('Wall time since last received command [s]');ax.set_ylabel('Speed [m/s]');ax.set_title('Watchdog trigger and physical stopping are different');ax.grid(alpha=.3);ax.legend(fontsize=8);fig.tight_layout();fig.savefig(p/'stopping.png',dpi=180);plt.close(fig)
print('Plots saved')
