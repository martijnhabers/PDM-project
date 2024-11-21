from setuptools import find_packages, setup

package_name = 'world_creator'

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
    maintainer='cdedood',
    maintainer_email='C.J.deDood@student.tudelft.nl',
    description='script that makes Gazebo world with randomly placed and sized objects',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'creator = creator.generate_world:main'
        ],
    },
)
