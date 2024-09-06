from setuptools import setup

package_name = 'i2c_devices_pkg'

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
    maintainer_email='axelray@post.bgu.ac.il',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "i2c_meas_node = i2c_devices_pkg.i2c_meas_node:main",
            "i2c_devices_publisher_node = i2c_devices_pkg.i2c_devices_publisher_node:main",
        ],
    },
)
