from setuptools import find_packages, setup

package_name = 'prm_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['prm_navigation/graph.gml']),
        ('share/' + package_name, ['prm_navigation/graph.json']),
        ('share/' + package_name, ['prm_navigation/occupancy_grid.csv']),
        # add launch file, locaetd in the launch directory of the package
        ('share/' + package_name + '/launch', ['launch/prm.launch.py']),

    ],
    install_requires=['setuptools', 'networkx'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prm_node = prm_navigation.prm_node:main',
            'visualization_node = prm_navigation.visualization_node:main',
        ],
    },
)
