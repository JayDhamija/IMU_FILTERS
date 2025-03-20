from setuptools import find_packages, setup

package_name = 'imu_filters'

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
    maintainer='jay',
    maintainer_email='jay@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter_node = imu_filters.filter_node:main',
            'butterworth_filter_order_comparison = imu_filters.butterworth_filter_order_comparison:main',
            'butterworth_filter_type_comparison = imu_filters.butterworth_filter_type_comparison:main',
            'msgs_conversion = imu_filters.msgs_conversion:main',  # Changed entry name here
        ],
    },
)
