from setuptools import setup
import glob

package_name = 'RoboticsProject2022'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usi',
    maintainer_email='granda@usi.ch,silva@usi.ch',
    description='Robotics 2022 project',
    license='MIT LIcense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
