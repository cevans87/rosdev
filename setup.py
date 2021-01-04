from setuptools import find_packages, setup

setup(
    author='Cameron Evans',
    author_email='c.d.evans87@gmail.com',
    description='CLI for common ROS developer tasks',
    entry_points={
        'console_scripts': ['rosdev=rosdev.__main__:main']
    },
    include_package_data=True,
    install_requires=[
        'argcomplete',
        'atools',
    ],
    license='mit',
    name='rosdev',
    packages=find_packages(),
    python_requires='>=3.8',
    url='https://github.com/cevans87/rosdev',
    version='0.1.0',
)
