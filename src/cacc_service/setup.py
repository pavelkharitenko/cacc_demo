from setuptools import setup

package_name = 'cacc_service'

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
    maintainer='maintainername',
    maintainer_email='your@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cacc_service = cacc_service.cacc_service_node:main',
            'obu_sensor = cacc_service.obu_sensor_node:main',
            'vehicle_model = cacc_service.vehicle_dynamics:main',
        ],
    },
)
