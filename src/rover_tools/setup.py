from setuptools import setup

package_name = 'rover_tools'

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
    maintainer='KASO',
    maintainer_email='petar.kaselj.00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualizer_node = rover_tools.visualizer_node:main',
            'simulator_node = rover_tools.simulator_node:main',
            'gauss_simulator_node = rover_tools.gauss_simulator_node:main',
        ],
    },
)
