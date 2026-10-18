from setuptools import setup

package_name = 'dqn_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'torch'],
    zip_safe=True,
    maintainer='TurtleBot3 Assignment',
    maintainer_email='student@example.com',
    description='DQN goal-directed controller for TurtleBot3 Burger.',
    license='MIT',
    entry_points={'console_scripts': ['dqn_controller = dqn_navigation.ros_node:main']},
)
