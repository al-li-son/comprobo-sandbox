from setuptools import find_packages, setup

package_name = 'example_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch_nodes.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali1',
    maintainer_email='allisonli18710@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = example_pkg.publisher:main',
            'subscriber = example_pkg.subscriber:main'
        ],
    },
)
