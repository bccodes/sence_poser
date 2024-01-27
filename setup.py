from setuptools import find_packages, setup

package_name = 'sence_poser'

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
    maintainer='Ben Worth',
    maintainer_email='u3243222@uni.canberra.edu.au',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'trajectory_pub = sence_poser.trajectory_pub:main',
                'action_test = sence_poser.action_test:main',
                'pose_action_server = sence_poser.pose_action_server:main',
        ],
    },
)
