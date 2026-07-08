from setuptools import find_packages, setup

package_name = 'imu_guard'

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
    maintainer='gemstone',
    maintainer_email='gemstone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['deneyap_bridge = imu_guard.deneyap_bridge:main',
    'imu_validator = imu_guard.imu_validator:main',
            'gemstone_imu_calibrator = imu_guard.gemstone_imu_calibrator:main',
        ],
    },
)
