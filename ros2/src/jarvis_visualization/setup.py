from setuptools import setup

package_name = 'jarvis_visualization'

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
    maintainer='mrsang',
    maintainer_email='mrsang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camviewer = jarvis_visualization.camera_viewer:main',
            'imuviewer = jarvis_visualization.imu_viewer:main',
        ],
    },
)
