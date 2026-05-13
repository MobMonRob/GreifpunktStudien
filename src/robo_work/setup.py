from setuptools import find_packages, setup

package_name = 'robo_work'

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
    maintainer='lukas',
    maintainer_email='XX',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = robo_work.client_controller:main',
            'is_part_present = robo_work.service_part_present:main',
            'get_grasp_point = robo_work.service_get_grasp_point:main',
            'check_label = robo_work.service_check_label:main'
        ],
    },
)
