import glob

from setuptools import setup

package_name = 'turtlebot3_dqn'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=['Gilbert', 'Ryan Shim', 'ChanHyeong Lee'],
    author_email=['kkjong@robotis.com', 'jhshim@robotis.com', 'dddoggi1207@gmail.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'ROS 2 package for turtlebot3_dqn.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'dqn_agent = turtlebot3_dqn.dqn_agent.dqn_agent:main',
            'dqn_environment = turtlebot3_dqn.dqn_environment.dqn_environment:main',
            'dqn_gazebo = turtlebot3_dqn.dqn_gazebo.dqn_gazebo:main',
            'dqn_test = turtlebot3_dqn.dqn_test.dqn_test:main',
            'result_graph = turtlebot3_dqn.utils.result_graph:main',
            'action_graph = turtlebot3_dqn.utils.action_graph:main',
        ],
    },
)
