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
        ('share/' + package_name + '/test', ['test/WorldListExport.py']),  # Ensure the file is copied to the correct directory
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
            'creator = world_creator.creator:main',  # Adjusted for creator.py
            'trajectory_builder = world_creator.line_creator:main',  # Adjusted for line_creator.py
        ],
    },
)
