from setuptools import setup
from glob import glob
from os.path import isfile
setup(name='ackermann_sim', version='0.1.0', packages=['ackermann_sim'],
 data_files=[('share/ament_index/resource_index/packages',['resource/ackermann_sim']),
 ('share/ackermann_sim',['package.xml']), *[('share/ackermann_sim/'+d,[p for p in glob(d+'/*') if isfile(p)]) for d in ['config','launch','rviz','vendor']]],
 install_requires=['setuptools'], zip_safe=True,
 maintainer='Project developer', maintainer_email='local@example.invalid', description='Minimal Ackermann simulation', license='Apache-2.0',
 entry_points={'console_scripts':['command_guard = ackermann_sim.guard:main','path_observer = ackermann_sim.observer:main']})
