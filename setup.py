from setuptools import find_packages, setup

package_name = 'vx_traffic_signal_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rav4243s@hs-coburg.de',
    maintainer_email='rav4243s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_signal_decision_node = vx_traffic_signal_decision.traffic_signal_decision_node:main',
            'dummy = vx_traffic_signal_decision.dummy_publisher:main'
        ],
    },
)
