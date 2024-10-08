from setuptools import find_packages, setup

package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch directory
        ('share/' + package_name + '/launch', [
            'launch/detection.launch.py',
            # Add any other launch files here if needed
        ]),
        # Install the config directory if you have one
        ('share/' + package_name + '/config', [
            'config/config.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sami',
    maintainer_email='samitrad7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_detection_node =detection.can_detection:main',
	        'qr_detection_node = detection.qr_detection:main',
            'main_node = detection.main:main',
        ],
    },
)
