from setuptools import setup

package_name = 'task02'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Publisher node example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = task02.publisher:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/task02.launch']),  # <-- Include the launch file
        ('share/' + package_name + "/config", ['config/task02.yaml']),  # <-- Include the config file
    ],
)
