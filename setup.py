from setuptools import find_packages, setup

setup(
    name='rosdev',
    version='0.1.0',
    packages=find_packages(),
    entry_points={
        'console_scripts': ['rosdev=rosdev.__init__:main']
    },
    install_requires=[
        'atools>=0.4.1',
    ],
    python_requires='>=3.7',
    url='https://github.com/cevans87/rosdev',
    license='mit',
    author='Cameron Evans',
    author_email='c.d.evans87@gmail.com',
    description='CLI for common ROS developer tasks'
)
