from setuptools import find_packages, setup

package_name = 'offboard_ctrl_node'

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
    maintainer='root',
    maintainer_email='68436564+theunknowninfinite@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_ctrl_node = offboard_ctrl_node.offboard_ctrl_node:main',
                # 'node_test= offboard_ctrl_node.node:main'
        ],
    },
)
