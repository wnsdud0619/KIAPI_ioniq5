from setuptools import setup

package_name = 'fake_localization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiapi',
    maintainer_email='kjyangks@naver.com',
    description='Fake localization node for Autoware Universe',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_localization_node = fake_localization_pkg.fake_localization_node:main'
        ],
    },
)

