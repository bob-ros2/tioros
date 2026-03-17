from setuptools import setup
from glob import glob
import os

package_name = 'tioros'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*[yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bob Ros',
    maintainer_email='bobmeuchelm@gmail.com',
    description='Twitch-ROS chatbot node package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
                'chatbot = tioros.chatbot:main',
                'filter = tioros.filter:main',
                'eventsub = tioros.eventsub:main',
                'auth = tioros.auth:main',
        ],
    },
)
