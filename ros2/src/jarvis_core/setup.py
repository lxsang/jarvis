from setuptools import setup

package_name = 'jarvis_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/core.py']),
        ('share/' + package_name, ['launch/vision.py']),
        ('share/' + package_name, ['launch/navigation.py']),
        ('share/' + package_name, ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jarvis',
    maintainer_email='jarvis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetty = jarvis_core.jetty:main',
            'stat = jarvis_core.stat:main',
            'mmetric = jarvis_core.mmetric:main',
            'camera = jarvis_core.camera:main',
            't2mvel = jarvis_core.t2mvel:main',
            'wv2pwm = jarvis_core.wv2pwm:main',
            'camviewer = jarvis_core.camera_viewer:main',
            'odometry = jarvis_core.odometry:main'
        ],
    },
)
