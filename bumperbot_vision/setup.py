from setuptools import find_packages, setup

package_name = 'bumperbot_vision'

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
    maintainer='utk',
    maintainer_email='kutkarsh706@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_maze_solver = bumperbot_vision.qr_maze_solver:main',
            'line_follower_real = bumperbot_vision.line_follower_real:main',
            'line_follower = bumperbot_vision.line_follower:main'
        ],
    },
)
