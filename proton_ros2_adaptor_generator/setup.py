from setuptools import find_packages, setup

package_name = 'proton_ros2_adaptor_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML', 'Jinja2'],
    zip_safe=True,
    maintainer='Tom Wallis',
    maintainer_email='thomas.wallis@rockwellautomation.com',
    description='Proton ROS 2 message bridge package generator',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
