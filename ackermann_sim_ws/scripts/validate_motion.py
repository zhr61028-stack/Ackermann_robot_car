#!/usr/bin/env python3
"""Drive only the isolated virtual vehicle; collect independent physical truth."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import cv2
import numpy as np
import yaml
from window_capture import AppWindowCapture
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from bag_safety import update_report


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def stamp(h):
    return h.stamp.sec+h.stamp.nanosec*1e-9


class Recorder(Node):
    def __init__(self):
        super().__init__('motion_validation')
        self.data={key:[] for key in ['odom','truth','joints','watchdog','safe','tf_static']}
        self.pub=self.create_publisher(Twist,'/cmd_vel',1)
        self.phase_pub=self.create_publisher(String,'/sim/test_phase',10)
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(TFMessage,'/ground_truth/poses',self.truth,10)
        self.create_subscription(JointState,'/joint_states',self.joints,1)
        self.create_subscription(String,'/sim/watchdog',lambda m:self.data['watchdog'].append(json.loads(m.data)),10)
        self.create_subscription(Twist,'/sim/cmd_vel',lambda m:self.data['safe'].append(dict(t=time.monotonic(),v=m.linear.x,w=m.angular.z)),10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage,'/tf_static',self.static,qos)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)

    def odom(self,m):
        self.data['odom'].append(dict(t=time.monotonic(),sim=stamp(m.header),x=m.pose.pose.position.x,y=m.pose.pose.position.y,
            yaw=yaw(m.pose.pose.orientation),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,
            frame=m.header.frame_id,child=m.child_frame_id))

    def truth(self,m):
        for tr in m.transforms:
            if tr.child_frame_id=='ackermann':
                self.data['truth'].append(dict(t=time.monotonic(),sim=stamp(tr.header),x=tr.transform.translation.x,
                    y=tr.transform.translation.y,yaw=yaw(tr.transform.rotation),z=tr.transform.translation.z,
                    frame=tr.header.frame_id,child=tr.child_frame_id))

    def joints(self,m):
        if not self.data['joints'] or stamp(m.header)-self.data['joints'][-1]['sim']>=0.019:
            self.data['joints'].append(dict(t=time.monotonic(),sim=stamp(m.header),positions=dict(zip(m.name,m.position))))

    def static(self,m):
        self.data['tf_static'].extend(dict(parent=t.header.frame_id,child=t.child_frame_id) for t in m.transforms)


class Video:
    def __init__(self,path):
        self.capture=AppWindowCapture()
        tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True).stdout
        self.windows={}
        for title in ['Gazebo Sim','RViz']:
            # Capture only the app-owned window, excluding unrelated desktop surfaces.
            candidates=[l for l in tree.splitlines() if ('("gz-sim-gui" "Gazebo GUI")' in l if title=='Gazebo Sim' else '("rviz2" "rviz2")' in l and 'ackermann.rviz' in l)]
            if candidates:self.windows[title]=int(re.search(r'0x[0-9a-fA-F]+',candidates[0]).group(),16)
        if len(self.windows)!=2:raise RuntimeError(f'Both GUI windows required: {self.windows}')
        self.output_directory=Path(path).parent
        self.codec='avc1';self.writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*self.codec),10,(1280,720))
        if not self.writer.isOpened():
            self.codec='mp4v';self.writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*self.codec),10,(1280,720))
        if not self.writer.isOpened():raise RuntimeError('No existing MP4 encoder available')
        self.frames=0

    def frame(self,phase,seconds,node):
        canvas=np.full((720,1280,3),28,dtype=np.uint8)
        for i,title in enumerate(['Gazebo Sim','RViz']):
            raw=self.capture.grab(self.windows[title])
            height,width=raw.shape[:2]
            scale=min(630/width,590/height);small=cv2.resize(raw,(int(width*scale),int(height*scale)))
            x=i*640+(640-small.shape[1])//2;y=55+(590-small.shape[0])//2
            canvas[y:y+small.shape[0],x:x+small.shape[1]]=small
            cv2.putText(canvas,title,(i*640+20,40),cv2.FONT_HERSHEY_SIMPLEX,.8,(230,230,230),2)
        cv2.putText(canvas,f'SIMULATION ASSUMPTIONS | {phase} | {seconds:.1f}s',(20,670),cv2.FONT_HERSHEY_SIMPLEX,.65,(100,220,255),2)
        if node.data['truth']:
            p=node.data['truth'][-1];s=node.data['safe'][-1] if node.data['safe'] else {'v':0,'w':0}
            cv2.putText(canvas,f'Truth x={p["x"]:.2f} y={p["y"]:.2f} yaw={p["yaw"]:.2f} | safe v={s["v"]:.2f} w={s["w"]:.2f}',(20,705),cv2.FONT_HERSHEY_SIMPLEX,.55,(220,220,220),1)
        self.writer.write(canvas);self.frames+=1
        if self.frames in [1,120,300,360]:cv2.imwrite(str(self.output_directory/f'preview-{self.frames}.png'),canvas)

    def close(self):self.writer.release();self.capture.close()


def circle(points):
    x=np.array([p['x'] for p in points]);y=np.array([p['y'] for p in points])
    a=np.c_[2*x,2*y,np.ones(len(x))]
    cx,cy,c=np.linalg.lstsq(a,x*x+y*y,rcond=None)[0]
    radius=math.sqrt(c+cx*cx+cy*cy)
    residual=float(np.sqrt(np.mean((np.hypot(x-cx,y-cy)-radius)**2)))
    return dict(radius_m=radius,fit_rms_m=residual,points=len(x))


def summarize(node,phases,config,tf_result):
    report={'assumptions':config,'tf':tf_result,'phases':{},'measurement_note':'Wall timings use monotonic receive timestamps. Physical speed uses independent model-pose differences in simulation time; observation sampling and transport delay remain.'}
    criteria=config['validation']
    for phase in phases:
        name=phase['name'];begin,end=phase['begin'],phase['end']
        truth=[p for p in node.data['truth'] if begin<=p['t']<=end]
        odom=[p for p in node.data['odom'] if begin<=p['t']<=end]
        joints=[p for p in node.data['joints'] if begin+1.5<=p['t']<=end]
        info=dict(duration_wall_s=end-begin,command_v=phase['v'],command_w=phase['w'],truth_samples=len(truth))
        if truth:
            info['distance_endpoint_m']=math.hypot(truth[-1]['x']-truth[0]['x'],truth[-1]['y']-truth[0]['y'])
            info['yaw_change_rad']=float(np.unwrap([p['yaw'] for p in truth])[-1]-truth[0]['yaw'])
        if name in ['left','right','limit_left']:
            stable=[p for p in truth if begin+2<=p['t']<=end-.1]
            stable_odom=[p for p in odom if begin+2<=p['t']<=end-.1]
            info['truth_circle']=circle(stable);info['odom_circle']=circle(stable_odom)
            info['left_steer_rad']=float(np.median([p['positions']['front_left_wheel_steering_joint'] for p in joints]))
            info['right_steer_rad']=float(np.median([p['positions']['front_right_wheel_steering_joint'] for p in joints]))
            safe=[p for p in node.data['safe'] if begin+1<=p['t']<=end]
            info['safe_v_mps']=float(np.median([p['v'] for p in safe]));info['safe_w_radps']=float(np.median([p['w'] for p in safe]))
            info['requested_radius_m']=abs(info['safe_v_mps']/info['safe_w_radps'])
        report['phases'][name]=info
    interruption=next(p for p in phases if p['name']=='interruption')
    expired=[p for p in node.data['watchdog'] if p['reason']=='timeout' and interruption['begin']<=p['monotonic_s']<=interruption['end']]
    report['watchdog']={'measurement_source':'Live GUI-process diagnostic only; final verdict requires complete bag audit'}
    if expired:
        first=expired[0]
        report['watchdog'].update(trigger_latency_wall_s=first['monotonic_s']-first['last_rx_monotonic_s'])
    # Compare nearest timestamp physical truth and wheel odometry, avoiding arrival-time offsets.
    times=np.array([p['sim'] for p in truth]);errors=[]
    for p in node.data['odom']:
        k=int(np.argmin(abs(times-p['sim'])))
        if abs(times[k]-p['sim'])<=.025:errors.append(math.hypot(p['x']-truth[k]['x'],p['y']-truth[k]['y']))
    report['odom_truth_max_position_error_m']=max(errors)
    p=report['phases'];tol=criteria['trajectory_radius_relative_tolerance']
    checks={
        'forward':p['forward']['distance_endpoint_m']>1 and abs(p['forward']['yaw_change_rad'])<.03,
        'explicit_stop':p['stop_forward']['distance_endpoint_m']<.10,
        'left_direction':p['left']['yaw_change_rad']>.5 and p['left']['left_steer_rad']>p['left']['right_steer_rad']>0,
        'right_direction':p['right']['yaw_change_rad']<-.5 and p['right']['right_steer_rad']<p['right']['left_steer_rad']<0,
        'left_radius':abs(p['left']['truth_circle']['radius_m']/p['left']['requested_radius_m']-1)<tol,
        'right_radius':abs(p['right']['truth_circle']['radius_m']/p['right']['requested_radius_m']-1)<tol,
        'minimum_radius':abs(p['limit_left']['truth_circle']['radius_m']/p['limit_left']['requested_radius_m']-1)<tol,
        'no_in_place_rotation':p['pure_rotation']['distance_endpoint_m']<.02 and abs(p['pure_rotation']['yaw_change_rad'])<.02,
        'speed_limit':p['limit_left']['safe_v_mps']<=config['vehicle']['max_speed_mps']+1e-9,
        'odom_truth_consistency':max(errors)<criteria['odom_truth_position_tolerance_m'],
        'watchdog_trigger':False, # Filled by complete-bag audit after the recorder closes.
        'physical_stop_observed':False,
        'tf_consistency':tf_result['pass']}
    report['checks']=checks;report['all_pass']=all(checks.values())
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    ws=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((ws/'src/ackermann_sim/config/vehicle.yaml').read_text())
    rclpy.init();node=Recorder();video=None;bag=None;bag_log=None;phases=[]
    try:
        deadline=time.monotonic()+10
        while (not node.data['odom'] or not node.data['truth'] or not node.data['watchdog']) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if not all(node.data[k] for k in ['odom','truth','watchdog']):raise RuntimeError('Simulator observations missing')
        bag_log=(output/'bag-record.log').open('w')
        bag=subprocess.Popen(['ros2','bag','record','-o',str(output/'bag'),'/clock','/cmd_vel','/sim/cmd_vel','/sim/watchdog','/odom','/tf','/tf_static','/joint_states','/ground_truth/poses','/sim/test_phase'],stdout=bag_log,stderr=subprocess.STDOUT,start_new_session=True)
        video=Video(output/'demo.mp4')
        sequence=[('settle',2,0,0),('forward',3,.5,0),('stop_forward',1.5,0,0),('left',8,.5,.2),('stop_left',1,0,0),
            ('right',8,.5,-.2),('stop_right',1,0,0),('pure_rotation',2,0,1),('limit_left',5,.9,.9),
            ('stop_limit',1,0,0),('interrupt_drive',2,.5,0),('interruption',2,None,None)]
        overall=time.monotonic();next_frame=overall;next_command=overall
        for name,duration,v,w in sequence:
            begin=time.monotonic();node.phase_pub.publish(String(data=name));print('PHASE',name,flush=True)
            while time.monotonic()-begin<duration:
                rclpy.spin_once(node,timeout_sec=.002);now=time.monotonic()
                if v is not None and now>=next_command:
                    m=Twist();m.linear.x=float(v);m.angular.z=float(w);node.pub.publish(m);next_command=now+.025
                if now>=next_frame:
                    video.frame(name,now-overall,node);next_frame+=.1
            phases.append(dict(name=name,begin=begin,end=time.monotonic(),v=v,w=w))
        node.pub.publish(Twist())
        tf_result={'pass':False}
        try:
            tr=node.buffer.lookup_transform('odom','base_footprint',rclpy.time.Time())
            last=node.data['odom'][-1]
            error=math.hypot(tr.transform.translation.x-last['x'],tr.transform.translation.y-last['y'])
            for frame in ['base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']:
                node.buffer.lookup_transform('odom',frame,rclpy.time.Time())
            tf_result={'pass':error<.03,'odom_tf_position_difference_m':error,'checked_frames':['base_footprint','base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']}
        except Exception as e:tf_result={'pass':False,'error':str(e)}
        report=summarize(node,phases,config,tf_result)
        report['video']={'codec':video.codec,'frames':video.frames,'fps':10,'duration_s':video.frames/10}
        (output/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        (output/'observations.json').write_text(json.dumps(node.data))
        print('Live capture complete; final timing will be audited from the full bag.',flush=True)
    finally:
        node.pub.publish(Twist())
        if video:video.close()
        if bag:
            os.killpg(bag.pid,signal.SIGINT)
            try:bag.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(bag.pid,signal.SIGTERM);bag.wait(timeout=5)
        if bag_log:bag_log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    # GUI capture can delay this process's subscription callbacks. The separate
    # bag recorder preserves the complete watchdog stream for the final verdict.
    report=update_report(output/'results.json')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    if not report['all_pass']:raise AssertionError('At least one motion criterion failed')


if __name__=='__main__':main()
