from setuptools import find_packages, setup

setup(
    name='rosdev',
    version='0.1.0',
    packages=find_packages(),
    entry_points={
        'console_scripts': ['rosdev=rosdev.__main__:main']
    },
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
        'stringcase>=1.2.0',
        'vcstool>=0.1.38'
    ],
    test_suite='tests',
    python_requires='>=3.7',
    url='https://github.com/cevans87/rosdev',
    license='mit',
    author='Cameron Evans',
    author_email='c.d.evans87@gmail.com',
    description='CLI for common ROS developer tasks'
)
