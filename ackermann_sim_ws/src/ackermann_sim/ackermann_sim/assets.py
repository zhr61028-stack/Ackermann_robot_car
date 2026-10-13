"""Parameterize the installed Gazebo official example; no new physics controller."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml


def put(parent, tag, text):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(text)
    return element


def validate(config):
    v = config['vehicle']
    for key, value in v.items():
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be positive and finite')
    angle = v['center_steering_limit_rad']
    if not 0 < angle < math.pi / 4:
        raise ValueError('Native plugin conversion requires center angle < pi/4')
    radius = v['wheelbase_m'] / math.tan(angle)
    inner = math.atan(v['wheelbase_m'] / (radius - v['track_m'] / 2))
    if radius <= v['track_m'] / 2 or inner > v['wheel_joint_limit_rad']:
        raise ValueError('Center steering limit exceeds physical wheel joint limit')
    c = config['control']
    if not 0 < c['timer_period_s'] < c['timeout_s']:
        raise ValueError('Invalid watchdog timing')
    return radius


def generate(share, config_path, output):
    share, output = Path(share), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(Path(config_path).read_text())
    validate(config)
    v, sim = config['vehicle'], config['simulation']
    length, track, radius = v['wheelbase_m'], v['track_m'], v['wheel_radius_m']
    tree = ET.parse(share / 'vendor/ackermann_steering.sdf')
    world = tree.getroot().find('world'); world.set('name', 'ackermann_test')
    put(world.find('physics'), 'max_step_size', sim['step_size_s'])
    model = world.find("model[@name='vehicle_blue']"); model.set('name', 'ackermann')
    put(model, 'pose', f'0 0 {radius + 0.005} 0 0 0')
    body = model.find("link[@name='chassis']"); body.set('name', 'base_link')
    put(body, 'pose', '0 0 0 0 0 0')
    center = f"{v['body_center_x_m']} 0 {v['body_center_z_m']} 0 0 0"
    size = f"{v['body_length_m']} {v['body_width_m']} {v['body_height_m']}"
    for kind in ['visual', 'collision']:
        put(body.find(kind), 'pose', center)
        put(body.find(kind + '/geometry/box'), 'size', size)
    inertial = body.find('inertial'); put(inertial, 'pose', center)
    mass = v['body_mass_kg']; put(inertial, 'mass', mass)
    inertia = inertial.find('inertia')
    a,b,c = v['body_length_m'],v['body_width_m'],v['body_height_m']
    for key, value in [('ixx',mass*(b*b+c*c)/12),('iyy',mass*(a*a+c*c)/12),('izz',mass*(a*a+b*b)/12)]:
        put(inertia,key,value)
    for side, sign in [('left',1),('right',-1)]:
        for end, x in [('front',length),('rear',0)]:
            wheel = model.find(f"link[@name='{end}_{side}_wheel']")
            put(wheel,'pose',f'{x} {sign*track/2} 0 {-math.pi/2} 0 0')
            mass=v['wheel_mass_kg']; wi=wheel.find('inertial');put(wi,'mass',mass)
            for key,value in [('ixx',mass*(3*radius**2+v['wheel_width_m']**2)/12),('iyy',mass*(3*radius**2+v['wheel_width_m']**2)/12),('izz',mass*radius**2/2)]:
                put(wi.find('inertia'),key,value)
            for cyl in wheel.findall('.//cylinder'):
                put(cyl,'radius',radius);put(cyl,'length',v['wheel_width_m'])
            for mu in wheel.findall('.//ode/mu')+wheel.findall('.//ode/mu2'):
                mu.text=str(v['tire_friction'])
        steering=model.find(f"link[@name='front_{side}_wheel_steering_link']")
        put(steering,'pose',f'{length} {sign*track/2} 0 0 0 0')
        put(steering.find('inertial'),'mass',v['steering_link_mass_kg'])
        for key in ['ixx','iyy','izz']:put(steering.find('inertial/inertia'),key,0.0001)
        for cyl in steering.findall('.//cylinder'):put(cyl,'radius',0.01);put(cyl,'length',0.04)
    for parent in model.findall('joint/parent'):
        if parent.text=='chassis':parent.text='base_link'
    for joint in model.findall('joint'):
        if 'steering_joint' in joint.get('name'):
            limit=joint.find('axis/limit')
            put(limit,'lower',-v['wheel_joint_limit_rad']);put(limit,'upper',v['wheel_joint_limit_rad'])
            put(limit,'velocity',2.0)
    control=model.find("plugin[@name='gz::sim::systems::AckermannSteering']")
    # Rear-wheel drive: passive front wheels avoid forcing the wrong arc speed.
    for tag in ['left_joint','right_joint']:
        for node in list(control.findall(tag)):control.remove(node)
    put(control,'left_joint','rear_left_wheel_joint');put(control,'right_joint','rear_right_wheel_joint')
    for key,value in {'kingpin_width':track,'wheel_base':length,'wheel_separation':track,'wheel_radius':radius,
        'steering_limit':math.asin(math.tan(v['center_steering_limit_rad'])),
        'steer_p_gain':v['steering_gain'],'min_velocity':-v['max_speed_mps'],'max_velocity':v['max_speed_mps'],
        'min_acceleration':-v['max_acceleration_mps2'],'max_acceleration':v['max_acceleration_mps2'],
        'topic':'/model/ackermann/cmd_vel','odom_topic':'/model/ackermann/odometry','tf_topic':'/model/ackermann/tf',
        'frame_id':'odom','child_frame_id':'base_footprint','odom_publish_frequency':sim['odom_frequency_hz']}.items():put(control,key,value)
    joint_plugin=ET.SubElement(model,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    put(joint_plugin,'topic','/model/ackermann/joint_state')
    pose_plugin=ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for key,value in {'publish_link_pose':'false','publish_nested_model_pose':'true','publish_model_pose':'true',
        'use_pose_vector_msg':'true','update_frequency':50}.items():put(pose_plugin,key,value)
    # Visual-only metre grid; collisions remain a single flat plane.
    grid=ET.SubElement(world,'model',name='metre_grid');put(grid,'static','true')
    gl=ET.SubElement(grid,'link',name='grid')
    for axis in [0,1]:
        for n in range(-10,16):
            visual=ET.SubElement(gl,'visual',name=f'line_{axis}_{n}')
            put(visual,'pose',f'{n if axis==0 else 2.5} {0 if axis==0 else n} 0.001 0 0 0')
            box=ET.SubElement(ET.SubElement(visual,'geometry'),'box')
            put(box,'size','0.012 25 0.001' if axis==0 else '25 0.012 0.001')
            material=ET.SubElement(visual,'material');put(material,'ambient','0.45 0.45 0.45 1');put(material,'diffuse','0.45 0.45 0.45 1')
    # All resources are local, so launch requires no Fuel downloads.
    ET.indent(tree);tree.write(output/'test_world.sdf',encoding='unicode',xml_declaration=True)
    urdf=ET.Element('robot',name='assumed_ackermann')
    ET.SubElement(urdf,'link',name='base_footprint')
    base=ET.SubElement(urdf,'link',name='base_link')
    j=ET.SubElement(urdf,'joint',name='footprint_to_base',type='fixed')
    ET.SubElement(j,'parent',link='base_footprint');ET.SubElement(j,'child',link='base_link');ET.SubElement(j,'origin',xyz=f'0 0 {radius}',rpy='0 0 0')
    def visual(link,geometry,xyz='0 0 0',rpy='0 0 0',color='0.15 0.35 0.85 1'):
        vi=ET.SubElement(link,'visual');ET.SubElement(vi,'origin',xyz=xyz,rpy=rpy)
        geo=ET.SubElement(vi,'geometry');ET.SubElement(geo,geometry[0],**geometry[1])
        ma=ET.SubElement(vi,'material',name=link.get('name')+'_color');ET.SubElement(ma,'color',rgba=color)
    visual(base,('box',{'size':size}),f"{v['body_center_x_m']} 0 {v['body_center_z_m']}")
    for side,sign in [('left',1),('right',-1)]:
        sl=ET.SubElement(urdf,'link',name=f'front_{side}_wheel_steering_link')
        sj=ET.SubElement(urdf,'joint',name=f'front_{side}_wheel_steering_joint',type='revolute')
        ET.SubElement(sj,'parent',link='base_link');ET.SubElement(sj,'child',link=sl.get('name'))
        ET.SubElement(sj,'origin',xyz=f'{length} {sign*track/2} 0',rpy='0 0 0');ET.SubElement(sj,'axis',xyz='0 0 1')
        ET.SubElement(sj,'limit',lower=str(-v['wheel_joint_limit_rad']),upper=str(v['wheel_joint_limit_rad']),effort='25',velocity='2')
        for end,x in [('front',length),('rear',0)]:
            link=ET.SubElement(urdf,'link',name=f'{end}_{side}_wheel')
            visual(link,('cylinder',{'radius':str(radius),'length':str(v['wheel_width_m'])}),color='0.12 0.12 0.12 1')
            j=ET.SubElement(urdf,'joint',name=f'{end}_{side}_wheel_joint',type='continuous')
            ET.SubElement(j,'parent',link=sl.get('name') if end=='front' else 'base_link');ET.SubElement(j,'child',link=link.get('name'))
            ET.SubElement(j,'origin',xyz='0 0 0' if end=='front' else f'0 {sign*track/2} 0',rpy=f'{-math.pi/2} 0 0');ET.SubElement(j,'axis',xyz='0 0 1')
    ET.indent(urdf);ET.ElementTree(urdf).write(output/'vehicle.urdf',encoding='unicode')
    bridges=[]
    for ros,gz,rt,gt,direction in [('/clock','/world/ackermann_test/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS'),
        ('/sim/cmd_vel','/model/ackermann/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ'),
        ('/odom','/model/ackermann/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
        ('/tf','/model/ackermann/tf','tf2_msgs/msg/TFMessage','gz.msgs.Pose_V','GZ_TO_ROS'),
        ('/joint_states','/model/ackermann/joint_state','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS'),
        ('/ground_truth/poses','/model/ackermann/pose','tf2_msgs/msg/TFMessage','gz.msgs.Pose_V','GZ_TO_ROS')]:
        bridges.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction=direction))
    (output/'bridge.yaml').write_text(yaml.safe_dump(bridges))
    return config
