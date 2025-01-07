from setuptools import setup

package_name = 'nav_pkg'

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
    maintainer='lej',
    maintainer_email='lej@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_nav_msg = nav_pkg.pub_nav_msg:main',
            'sub_n_go = nav_pkg.sub_n_go:main',
            'follow_waypoints1 = nav_pkg.follow_waypoints1:main',
            'follow_waypoints2 = nav_pkg.follow_waypoints2:main',
            'follow_waypoints3 = nav_pkg.follow_waypoints3:main',
            'follow_waypoints4 = nav_pkg.follow_waypoints4:main',
        ],
    },
)
