from setuptools import find_packages, setup

package_name = 'service_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sns',
    maintainer_email='syednazmussakib64@gmail.com',
    description='ROS 2 Service Demo - Learn Request/Response Communication',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'server = service_demo.server:main',
            'client = service_demo.client:main',
        ],
    },
)
