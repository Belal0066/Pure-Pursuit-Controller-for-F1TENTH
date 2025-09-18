from setuptools import setup

package_name = 'pure_pursuit_working'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pure_pursuit.launch.py']),
        ('share/' + package_name + '/config', ['config/pure_pursuit.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Working Pure Pursuit Controller for F1tenth',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit_working.pure_pursuit_node:main',
        ],
    },
)

