from setuptools import find_packages, setup

setup(
    author='Cameron Evans',
    author_email='c.d.evans87@gmail.com',
    data_files=[('', ['rosdev/gen/idea/c.kdbx'])],
    description='CLI for common ROS developer tasks',
    entry_points={
        'console_scripts': ['rosdev=rosdev.__main__:main']
    },
    include_package_data=True,
    install_requires=[
        'argcomplete>=1.9.4',
        'atools>=0.6.2',
        'docker>=3.7.0',
        'frozendict>=1.2',
        'lxml>=4.3.3',
        'pycryptodome>=3.8.1',
        'pykeepass>=2.19',
        'pytest>=4.4.1',
        'python-jenkins>=1.4.0',
        'vcstool>=0.1.38',
    ],
    license='mit',
    name='rosdev',
    packages=find_packages(),
    python_requires='>=3.7',
    test_suite='tests',
    url='https://github.com/cevans87/rosdev',
    version='0.1.0',
)
