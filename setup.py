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
        'atools>=0.5.0',
        'docker>=3.7.0',
        'pyperclip>=1.7.0',
        'python-jenkins>=1.4.0',
        'PyYAML>=3.13'
    ],
    python_requires='>=3.7',
    url='https://github.com/cevans87/rosdev',
    license='mit',
    author='Cameron Evans',
    author_email='c.d.evans87@gmail.com',
    description='CLI for common ROS developer tasks'
)
