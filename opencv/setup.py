from setuptools import setup

package_name = 'opencv'

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
            'grayscale      = opencv.grayscale:main',
            'grayscale2      = opencv.grayscale2:main',
            'grayscale3      = opencv.grayscale3:main',
            'get_blue      = opencv.get_blue:main',
            'track_blue      = opencv.track_blue:main',
            'track_blue2      = opencv.track_blue2:main',
        ],
    },
)
