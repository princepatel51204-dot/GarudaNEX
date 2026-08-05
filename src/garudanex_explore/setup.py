from setuptools import find_packages, setup

package_name = 'garudanex_explore'

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
    maintainer='prince',
    maintainer_email='princepatel51204@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_recorder = garudanex_explore.run_recorder:main',
            'smart_explorer = garudanex_explore.smart_explorer:main',
            'explorer = garudanex_explore.explorer_node:main',
        ],
    },
)
