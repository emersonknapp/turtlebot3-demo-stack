from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_joy_interpreter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'Nodes to interpret joystick output and turn into motion commands, for the Turtlebot3.',
    ),
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox360_interpreter = turtlebot3_joy_interpreter.interpreter:main',
        ],
    },
)
