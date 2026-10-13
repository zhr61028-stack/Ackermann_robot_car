"""Audit watchdog and physical stopping from the complete bag, not GUI callbacks."""
import json
import math
from pathlib import Path
import statistics
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def audit(bag,config,phase_name="interruption"):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    statuses=[];truth=[];phases=[];offsets=[]
    while reader.has_next():
        topic,data,epoch_ns=reader.read_next()
        if topic=='/sim/watchdog':
            x=json.loads(deserialize_message(data,String).data)
            statuses.append(x);offsets.append(epoch_ns/1e9-x['monotonic_s'])
        elif topic=='/sim/test_phase':
            phases.append((deserialize_message(data,String).data,epoch_ns/1e9))
        elif topic=='/ground_truth/poses':
            for tr in deserialize_message(data,TFMessage).transforms:
                if tr.child_frame_id=='ackermann':
                    q=tr.transform.rotation
                    truth.append(dict(epoch=epoch_ns/1e9,sim=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,
                        x=tr.transform.translation.x,y=tr.transform.translation.y,
                        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
    offset=statistics.median(offsets)
    begin=next(epoch-offset for name,epoch in phases if name==phase_name)
    first=next(x for x in statuses if x['monotonic_s']>=begin and x['reason']=='timeout')
    last_rx=first['last_rx_monotonic_s'];c=config['validation'];stationary=[]
    for prev,cur in zip(truth,truth[1:]):
        t=cur['epoch']-offset;dt=cur['sim']-prev['sim']
        if dt<=0 or t<last_rx:continue
        speed=math.hypot(cur['x']-prev['x'],cur['y']-prev['y'])/dt
        yaw_rate=abs(math.atan2(math.sin(cur['yaw']-prev['yaw']),math.cos(cur['yaw']-prev['yaw'])))/dt
        if speed<c['stop_speed_threshold_mps'] and yaw_rate<c['stop_yaw_rate_threshold_radps']:
            stationary.append(dict(t=t,sim=cur['sim'],speed=speed,yaw_rate=yaw_rate))
            if t-stationary[0]['t']>=c['stop_hold_s']:break
        else:stationary=[]
    if not stationary:raise AssertionError('Complete bag does not show a sustained physical stop')
    near=[x for x in statuses if last_rx-.1<=x['monotonic_s']<=last_rx+1]
    gaps=[b['monotonic_s']-a['monotonic_s'] for a,b in zip(near,near[1:])]
    result=dict(measurement_source='Complete MCAP bag; embedded guard monotonic timestamps; independent model-pose samples at bag-recorder receive time',
        timeout_config_s=config['control']['timeout_s'],last_rx_monotonic_s=last_rx,
        trigger_monotonic_s=first['monotonic_s'],trigger_latency_wall_s=first['monotonic_s']-last_rx,
        trigger_ros_s=first['ros_ns']/1e9,safe_v_at_trigger=first['safe_speed'],safe_w_at_trigger=first['safe_yaw_rate'],
        physical_stop_latency_wall_s=stationary[0]['t']-last_rx,
        physical_stop_confirmed_latency_wall_s=stationary[-1]['t']-last_rx,
        stop_after_trigger_wall_s=stationary[0]['t']-first['monotonic_s'],
        physical_stop_sim_s=stationary[0]['sim'],threshold_speed_mps=c['stop_speed_threshold_mps'],
        threshold_yaw_rate_radps=c['stop_yaw_rate_threshold_radps'],hold_s=c['stop_hold_s'],
        guard_max_tick_gap_near_interrupt_s=max(gaps),pose_nominal_sample_period_sim_s=.02,
        clock_alignment='Median bag receive epoch minus embedded watchdog monotonic time; includes residual transport/sampling uncertainty')
    return result


def update_report(path):
    path=Path(path);report=json.loads(path.read_text())
    report.setdefault('live_subscriber_watchdog_diagnostic',report['watchdog'])
    report['watchdog']=audit(path.parent/'bag',report['assumptions'])
    period=report['assumptions']['control']['timeout_s']
    report['checks']['watchdog_trigger']=period<=report['watchdog']['trigger_latency_wall_s']<.3
    report['checks']['physical_stop_observed']=report['watchdog']['physical_stop_latency_wall_s']<.3
    report['all_pass']=all(report['checks'].values())
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return report


if __name__=='__main__':
    import sys
    print(json.dumps(update_report(sys.argv[1])['watchdog'],indent=2))
