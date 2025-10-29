from setuptools import setup

package_name = 'onrobot_rg2ft_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gilberto López',
    maintainer_email='A01637071@tec.mx',
    description='Control del gripper OnRobot RG2FT desde ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onrobot_rg2ft_driver = onrobot_rg2ft_control.OnRobotRG2FTDriver:main',
            'control_gripper_terminal = onrobot_rg2ft_control.control_gripper_terminal:main',
        ],
    },
)

