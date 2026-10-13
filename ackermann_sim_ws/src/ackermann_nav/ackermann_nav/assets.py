"""Generate test world and occupancy image from one fixed-obstacle description."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
import numpy as np
import yaml
from ackermann_sim.assets import generate as vehicle_generate,put,validate

def generate(sim_share,nav_share,vehicle_path,scene_path,output):
 output=Path(output);nav_share=Path(nav_share)
 c=vehicle_generate(sim_share,vehicle_path,output);v=c['vehicle'];scene=yaml.safe_load(Path(scene_path).read_text())
 tree=E.parse(output/'test_world.sdf');world=tree.getroot().find('world');car=world.find("model[@name='ackermann']")
 for b in scene['obstacles']:
  m=E.SubElement(world,'model',name=b['name']);put(m,'static','true');put(m,'pose',f"{b['x']} {b['y']} {b['size_z']/2} 0 0 0")
  link=E.SubElement(m,'link',name='obstacle')
  for tag in ['collision','visual']:
   e=E.SubElement(link,tag,name='box');box=E.SubElement(E.SubElement(e,'geometry'),'box');put(box,'size',f"{b['size_x']} {b['size_y']} {b['size_z']}")
   if tag=='visual':
    mat=E.SubElement(e,'material');put(mat,'ambient','0.8 0.25 0.12 1');put(mat,'diffuse','0.8 0.25 0.12 1')
 E.SubElement(world,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
 bridges=yaml.safe_load((output/'bridge.yaml').read_text());bridges=[b for b in bridges if b['ros_topic_name']!='/tf']
 for b in bridges:
  if b['ros_topic_name']=='/odom':b['ros_topic_name']='/wheel/odom'
 # A second odom->base transform is never bridged: truth node is the sole owner.
 for link in car.findall('link'):
  if link.get('name')!='base_link' and not link.get('name').endswith('_wheel'):continue
  collision=link.find('collision');name=link.get('name');topic='/model/ackermann/contacts/'+name
  sensor=E.SubElement(link,'sensor',name='contacts_'+name,type='contact');put(sensor,'always_on','true');put(sensor,'update_rate',100)
  contact=E.SubElement(sensor,'contact');put(contact,'collision',collision.get('name'));put(contact,'topic',topic)
  bridges.append(dict(ros_topic_name='/sim/contacts/'+name,gz_topic_name=topic,ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='gz.msgs.Contacts',direction='GZ_TO_ROS'))
 E.indent(tree);tree.write(output/'test_world.sdf',encoding='unicode',xml_declaration=True);(output/'bridge.yaml').write_text(yaml.safe_dump(bridges))
 xmin,ymin,xmax,ymax=scene['bounds_m'];res=scene['resolution_m'];w=round((xmax-xmin)/res);h=round((ymax-ymin)/res)
 image=np.full((h,w),254,dtype=np.uint8);image[:2,:]=0;image[-2:,:]=0;image[:,:2]=0;image[:,-2:]=0
 for b in scene['obstacles']:
  x1=math.floor((b['x']-b['size_x']/2-xmin)/res);x2=math.ceil((b['x']+b['size_x']/2-xmin)/res)
  y1=math.floor((b['y']-b['size_y']/2-ymin)/res);y2=math.ceil((b['y']+b['size_y']/2-ymin)/res)
  image[h-y2:h-y1,x1:x2]=0
 (output/'test_map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+image.tobytes())
 (output/'test_map.yaml').write_text(yaml.safe_dump(dict(image='test_map.pgm',mode='trinary',resolution=res,origin=[xmin,ymin,0.0],negate=0,occupied_thresh=.65,free_thresh=.196)))
 params=yaml.safe_load((nav_share/'config/nav2.yaml').read_text())
 rear=min(v['body_center_x_m']-v['body_length_m']/2,-v['wheel_radius_m'])-.04
 front=max(v['body_center_x_m']+v['body_length_m']/2,v['wheelbase_m']+v['wheel_radius_m'])+.04
 half=max(v['body_width_m']/2,v['track_m']/2+math.hypot(v['wheel_radius_m'],v['wheel_width_m']/2))+.04
 footprint=[[rear,-half],[front,-half],[front,half],[rear,half]]
 for name in ['local_costmap','global_costmap']:
  p=dict(use_sim_time=True,global_frame='odom',robot_base_frame='base_footprint',resolution=res,footprint=str(footprint),footprint_padding=.02,
   update_frequency=10.0,publish_frequency=2.0,transform_tolerance=.2,track_unknown_space=True,always_send_full_costmap=True,
   plugins=['static_layer','inflation_layer'],static_layer={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True,'subscribe_to_updates':False,'map_topic':'/map'},
   inflation_layer={'plugin':'nav2_costmap_2d::InflationLayer','cost_scaling_factor':2.5,'inflation_radius':1.2})
  if name=='local_costmap':p.update(rolling_window=True,width=12,height=12)
  params[name]={name:{'ros__parameters':p}}
 physical_radius=validate(c);planner_radius=physical_radius*1.1
 params['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']=planner_radius
 params['map_server']['ros__parameters']['yaml_filename']=str(output/'test_map.yaml')
 params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(nav_share/'config/forward_only.xml')
 (output/'nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
 (output/'derived_parameters.yaml').write_text(yaml.safe_dump(dict(assumption_status=c['assumption_status'],physical_min_radius_m=physical_radius,planner_min_radius_m=planner_radius,footprint=footprint,localization='TEST_GAZEBO_TRUTH',navigation_frame='odom',scene=scene)))
 return params
