from setuptools import setup

package_name = 'dobot_cr10_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joey',
    maintainer_email='joey@example.com',
    description='Minimal Dobot CR10 robot description package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        # Package XML
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        # URDF files
        ('share/' + package_name + '/urdf', ['urdf/dobot_cr10.urdf.xacro']),
    ],
)

