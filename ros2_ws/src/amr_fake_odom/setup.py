from setuptools import find_packages, setup
package_name = 'amr_fake_odom'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Fake Odom',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['fake_odom_node = amr_fake_odom.fake_odom_node:main'],
    },
)