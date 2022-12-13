from setuptools import setup

package_name = 'iot_agent'

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
    maintainer='newslab-wayside',
    maintainer_email='r10944068@ntu.edu.tw',
    description='Final project agent',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iot_agent = iot_agent.agent:main',
        ],
    },
)
