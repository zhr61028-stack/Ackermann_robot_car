from pathlib import Path
import xml.etree.ElementTree as E
import yaml,numpy as np
from ackermann_nav.assets import generate as base_generate
from ackermann_sim.assets import put

def generate(sim,nav,share,vehicle,out,start):
 out=Path(out);roads=yaml.safe_load((share/'config/roads.yaml').read_text())
 scene=out/'scene.yaml';scene.write_text(yaml.safe_dump(dict(bounds_m=roads['bounds_m'],resolution_m=roads['resolution_m'],obstacles=[])))
 params=base_generate(sim,nav,vehicle,scene,out)
 tree=E.parse(out/'test_world.sdf');world=tree.getroot().find('world');car=world.find("model[@name='ackermann']")
 pose=car.find('pose').text.split();s=roads['starts'][start];pose[0]=str(s['x']);pose[1]=str(s['y']);pose[5]=str(s['yaw']);car.find('pose').text=' '.join(pose)
 for plugin in car.findall('plugin'):
  if plugin.find('odom_publish_frequency') is not None:
   f=plugin.find('frame_id')
   if f is not None:f.text='wheel_odom'
 for i,(x1,y1,x2,y2) in enumerate(roads['road_rectangles']):
  m=E.SubElement(world,'model',name=f'road_visual_{i}');put(m,'static','true');put(m,'pose',f'{(x1+x2)/2} {(y1+y2)/2} 0.002 0 0 0')
  l=E.SubElement(m,'link',name='surface');v=E.SubElement(l,'visual',name='asphalt');b=E.SubElement(E.SubElement(v,'geometry'),'box');put(b,'size',f'{x2-x1} {y2-y1} 0.004');mat=E.SubElement(v,'material');put(mat,'ambient','0.22 0.22 0.22 1');put(mat,'diffuse','0.22 0.22 0.22 1')
 E.indent(tree);tree.write(out/'test_world.sdf',encoding='unicode')
 xmin,ymin,xmax,ymax=roads['bounds_m'];r=roads['resolution_m'];w=round((xmax-xmin)/r);h=round((ymax-ymin)/r)
 xs=xmin+(np.arange(w)+.5)*r;ys=ymax-(np.arange(h)+.5)*r;free=np.zeros((h,w),bool)
 for x1,y1,x2,y2 in roads['road_rectangles']:free|=(ys[:,None]>=y1)&(ys[:,None]<=y2)&(xs[None,:]>=x1)&(xs[None,:]<=x2)
 (out/'test_map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+np.where(free,254,0).astype('uint8').tobytes())
 for name in ['controller_server','behavior_server']:pass
 (out/'nav2.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
 return roads
