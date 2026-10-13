import json,math,bisect,argparse
from pathlib import Path
import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message as dec
from nav_msgs.msg import Odometry,Path as NavPath,OccupancyGrid
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import String

a=argparse.ArgumentParser();a.add_argument('experiment');args=a.parse_args();out=Path(args.experiment);spec=json.loads((out/'测试结果/scenario.json').read_text());live=json.loads((out/'测试结果/live.json').read_text());r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));raw=[];goals=[];plans=[];states=[];timeouts=[];route=[];selected=[];request=[];odom=[];map_shape=None
while r.has_next():
 topic,d,t=r.read_next();t/=1e9
 if topic=='/route/nav_cmd_vel':m=dec(d,Twist);raw.append([t,m.linear.x,m.angular.z])
 elif topic=='/route/local_goal':goals.append(t)
 elif topic=='/plan':plans.append(t)
 elif topic=='/route/status':states.append([t,json.loads(dec(d,String).data)])
 elif topic=='/sim/watchdog':
  m=json.loads(dec(d,String).data)
  if m['expired']:timeouts.append(t)
 elif topic=='/route/path':route=[[p.pose.position.x,p.pose.position.y] for p in dec(d,NavPath).poses]
 elif topic=='/route/selected':selected.append(json.loads(dec(d,String).data))
 elif topic=='/route/request':request.append(json.loads(dec(d,String).data))
 elif topic=='/odom':
  m=dec(d,Odometry);odom.append([t,m.header.stamp.sec+m.header.stamp.nanosec/1e9,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z])
 elif topic=='/map':m=dec(d,OccupancyGrid);map_shape={'width':m.info.width,'height':m.info.height,'cells':len(m.data),'resolution':m.info.resolution}
graph=json.loads((out/'测试结果/graph.geojson').read_text());edges={f['properties']['id']:f['geometry']['coordinates'] for f in graph['features'] if f['geometry']['type']=='MultiLineString'}
length=sum(sum(math.dist(a,b) for a,b in zip(line,line[1:])) for eid in selected[-1]['edges'] for line in edges[eid]);smooth=sum(math.dist(a,b) for a,b in zip(route,route[1:]));stimes=[t for t,s in states]
active_timeout=[t for t in timeouts if states[max(0,bisect.bisect_right(stimes,t)-1)][1]['state']=='RUNNING'];gaps=[b[0]-a[0] for a,b in zip(raw,raw[1:])];latencies=[]
for t in plans:
 i=bisect.bisect_right(goals,t)-1
 if i>=0:latencies.append(t-goals[i])
resources=json.loads((out/'测试结果/resources.json').read_text());cpu=[]
for previous,current in zip(resources,resources[1:]):
 old={p['pid']:p['cpu_s'] for p in previous['processes']};cpu.append(100*sum(max(0,p['cpu_s']-old.get(p['pid'],p['cpu_s'])) for p in current['processes'])/(current['monotonic_s']-previous['monotonic_s']))
rtf=(resources[-1]['sim_time_s']-resources[0]['sim_time_s'])/(resources[-1]['monotonic_s']-resources[0]['monotonic_s']);pauses=[];begin=None;first=next(t for t,sim,x,y,v,w in odom if v>.05)
for t,sim,x,y,v,w in odom:
 eligible=t>=first and math.dist([x,y],spec['target'][:2])>1
 if eligible and abs(v)<.02:
  if begin is None:begin=t
 elif begin is not None:
  if t-begin>.2:pauses.append([begin,t,t-begin])
  begin=None
result={'graph_selected_length_m':length,'route_smoothed_length_m':smooth,'actual_odom_integrated_position_distance_m':sum(math.dist(a[2:4],b[2:4]) for a,b in zip(odom,odom[1:])),'start_to_goal_straight_distance_m':math.dist(live['initial'].values() if False else [live['initial']['x'],live['initial']['y']],spec['target'][:2]),'selected_route':selected,'final_goal_requests':request,'manual_interventions':live['manual_interventions'],'navigation_terminal_wall_s':live['navigation_terminal_wall_s'],'total_observation_wall_s':live['duration_wall_s'],'navigation_sim_time_s':odom[-1][1]-odom[0][1],'planning_requests':len(goals),'published_plans':len(plans),'planning_failed_attempts':len(goals)-len(plans),'planning_response_observed_s':{'mean':float(np.mean(latencies)),'p95':float(np.percentile(latencies,95)),'max':max(latencies)},'planning_timing_note':'goal topic received to plan topic received; includes DDS/action dispatch/scheduling, not pure algorithm CPU duration','raw_control_interval_s':{'mean':float(np.mean(gaps)),'p95':float(np.percentile(gaps,95)),'max':max(gaps),'above_60ms':sum(x>.06 for x in gaps)},'guard_expired_samples_while_running':len(active_timeout),'all_phase_intermediate_pauses':pauses,'map':map_shape,'resource':{'samples':len(resources),'aggregate_rss_peak_bytes':max(s['aggregate_rss_bytes'] for s in resources),'system_available_min_bytes':min(s['system_available_bytes'] for s in resources),'cpu_percent_mean':float(np.mean(cpu)),'cpu_percent_max':max(cpu),'cpu_note':'100%=one logical core; launched process group plus validator; RSS shared pages may duplicate','mean_sim_realtime_factor':rtf}}
result['checks']={'length':length>=spec['minimum_graph_length_m'],'autonomous_single_task':len(request)==1 and live['manual_interventions']==0,'no_planner_failure':len(goals)==len(plans),'no_intermediate_pause':not pauses,'no_running_watchdog_expiry':not active_timeout,'memory_available':result['resource']['system_available_min_bytes']>=2*1024**3};result['all_pass']=all(result['checks'].values());(out/'测试结果/long_metrics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2))
