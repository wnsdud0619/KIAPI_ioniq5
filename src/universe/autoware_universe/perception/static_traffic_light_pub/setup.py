from setuptools import setup

package_name = 'static_traffic_light_pub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiapi',
    maintainer_email='kiapi@example.com',
    description='Static traffic light publisher for Autoware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tl_publisher = static_traffic_light_pub.static_tl_publisher:main',
        ],
    },
)

