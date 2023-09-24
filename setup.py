from setuptools import setup
from glob import glob
import os

package_name = 'tioros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@bob.de',
    description='TwitchIO ROS Package',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'chatbot = tioros.chatbot:main',
        ],
    },
)
