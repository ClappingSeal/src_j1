from setuptools import setup

package_name = 'drone_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ddsdol',
    maintainer_email='ddsdol@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_node1 = drone_package.drone_node1:main',
            'drone_node2 = drone_package.drone_node2:main',
            'drone_node1_apf_ppo = drone_package.drone_node1_apf_ppo:main',
            'drone_node2_apf_ppo = drone_package.drone_node2_apf_ppo:main',
            'drone_node1_apf_td3 = drone_package.drone_node1_apf_td3:main',
            'drone_node2_apf_td3 = drone_package.drone_node2_apf_td3:main',
            'land_client1 = drone_package.land_client1:main',
            'land_client2 = drone_package.land_client2:main',
            'takeoff_client1 = drone_package.takeoff_client1:main',
            'takeoff_client2 = drone_package.takeoff_client2:main',
            'pos_sub = drone_package.pos_sub:main',
        ],
    },
)
