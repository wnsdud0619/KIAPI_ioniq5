from setuptools import find_packages, setup
import os               # 추가
from glob import glob   # 추가

package_name = 'custom_traffic_light_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- 아래 한 줄을 반드시 추가해야 launch 파일이 설치됩니다! ---
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiapi',
    maintainer_email='kiapi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_yolo_detector = custom_traffic_light_detector.traffic_light_yolo_detector:main'
        ],
    },
)
