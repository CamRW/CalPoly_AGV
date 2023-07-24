from setuptools import setup

package_name = 'realsense_obj_det'

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
    maintainer='admin',
    maintainer_email='crweigel@cpp.edu',
    description='Basic Realsense Obj Det',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_obj_det_node = realsense_obj_det.realsense_obj_det_node:main'
        ],
    },
)
