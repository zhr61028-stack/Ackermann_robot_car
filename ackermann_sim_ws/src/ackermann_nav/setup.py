from setuptools import setup
from glob import glob
from os.path import isfile
setup(name='ackermann_nav',version='0.1.0',packages=['ackermann_nav'],data_files=[('share/ament_index/resource_index/packages',['resource/ackermann_nav']),('share/ackermann_nav',['package.xml']),*[('share/ackermann_nav/'+d,[p for p in glob(d+'/*') if isfile(p)]) for d in ['config','launch','rviz']]],install_requires=['setuptools'],zip_safe=True,maintainer='Project developer',maintainer_email='local@example.invalid',description='M3 preliminary navigation, simulator truth only',license='Apache-2.0',entry_points={'console_scripts':['truth_localization = ackermann_nav.truth_localization:main']})
