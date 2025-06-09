from setuptools import find_packages, setup

package_name = 'pinocchio_ik_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ik_solver_slider.launch.py',]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grail',
    maintainer_email='szheng2@g.clemson.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slider_gui_pose = pinocchio_ik_solver.slider_gui_pose:main',
            'initial_pose = pinocchio_ik_solver.initial_pose_publisher:main',
            'ik_solver = pinocchio_ik_solver.clik_vr_ros:main'
            
        ],
    },
)
