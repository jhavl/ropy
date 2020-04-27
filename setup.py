from setuptools import setup, find_packages

setup(
    name='ropy',

    version='0.1',

    description='A Python library for robot control',

    long_description='ropy is a tool which can be used to interface with robos.',

    url='https://github.com/jhavl/ropy',

    author='Jesse Haviland',

    license='MIT',

    python_requires='>=2.7',

    packages=find_packages(),

    install_requires=['numpy', 'transforms3d']

)
