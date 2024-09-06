from setuptools import setup

package_name = 'autopilot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lar',
    maintainer_email='gridish@post.bgu.ac.il',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "qgc_comm_node = autopilot_pkg.qgc_comm_node:main",
            "parameters_node = autopilot_pkg.parameters_node:main",
            "mission_control_node = autopilot_pkg.mission_control_node:main",
            "guidance_node = autopilot_pkg.guidance_node:main",
            "live_node = autopilot_pkg.live_node:main",
        ],
    },
)
