from setuptools import setup

package_name = 'ebot_controller'

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
    maintainer='bhavesh kadodiya',
    maintainer_email='bhaveshkadodiya01@gmail.com',
    description='eBot autonomous navigation controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ebot_nav_task1A = ebot_controller.ebot_nav_task1A:main',
        ],
    },
)
