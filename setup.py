from setuptools import setup

package_name = 'turtlesim_pde4430'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='TurtleSim exercises for PDE4430',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_line = turtlesim_pde4430.straight_line:main',
            'circle = turtlesim_pde4430.circle:main',
            'figure8 = turtlesim_pde4430.figure8:main',
            'roomba = turtlesim_pde4430.roomba:main',
            'spawn_multi = turtlesim_pde4430.spawn_multi:main',
            'goto = turtlesim_pde4430.goto:main',
        ],
    },
)
