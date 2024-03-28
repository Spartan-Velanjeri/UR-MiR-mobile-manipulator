from setuptools import find_packages, setup

package_name = 'point_cloud_functions'

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
    maintainer='Parthan Manisekaran',
    maintainer_email='parthanvelanjeri@hotmail.com',
    description='Point Cloud related Python Scripts',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_cloud_reader = point_cloud_functions.point_cloud_reader:main'
        ],
    },
)
