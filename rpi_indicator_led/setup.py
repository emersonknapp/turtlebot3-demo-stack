import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'rpi_indicator_led'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}/launch'.format(package_name),
            glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Emerson Knapp',
    author_email='emerson.b.knapp@gmail.com',
    maintainer='AWS ROS Contributions',
    maintainer_email='aws-ros-contributions@amazon.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Nodes to run LEDs from Raspberry Pi GPIO, for headless application simple feedback.',
    ),
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_blinker = {}.indicator:main'.format(package_name),
        ],
    },
)
