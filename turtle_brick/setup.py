from setuptools import find_packages, setup

package_name = 'turtle_brick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 
                                   'launch/show_turtle.launch.xml',
                                   'urdf/turtle.urdf.xacro',
                                   'config/view_turtle.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sairamu02',
    maintainer_email='sairamumakanth2028@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
