from setuptools import setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/tracking.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dilly',
    maintainer_email='dilan.wijesinghe0@gmail.com',
    description='OpenCV package for tracking balloons',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'im_pub = tracking.im_pub:main',
            'im_sub = tracking.im_sub:main'
        ],
    },
)
