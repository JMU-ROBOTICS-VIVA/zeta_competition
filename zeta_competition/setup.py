from setuptools import setup
from glob import glob
import os

package_name = 'zeta_competition'

# model_data_files = []
# for cur_dir, _, files in os.walk('models'):
#     to = 'share/' + package_name + '/' + cur_dir + "/"
#     model_data_files.append((to, [cur_dir + "/" + f for f in files]))

data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch/', glob('launch/*.launch.py')),
    ('share/' + package_name + '/worlds/', glob('worlds/*.world')),
    ('share/' + package_name + '/maps/', glob('maps/*.yaml')),
    ('share/' + package_name + '/config/', glob('config/*.yaml')),
    ('share/' + package_name + '/config/', glob('config/*.csv')),
    ('share/' + package_name + '/rviz/', glob('rviz/*.rviz')),
    ('share/' + package_name + '/maps/', glob('maps/*.pgm'))]
#print(data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spragunr',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'report_button = zeta_competition.report_button:main',
            'victim_listener = zeta_competition.victim_listener:main',
            'set_initial_pose = zeta_competition.set_initial_pose:main',
            'zeta_scorer = zeta_competition.zeta_scorer:main',
        ],
    },
)
