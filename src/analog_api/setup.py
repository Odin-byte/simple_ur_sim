from setuptools import setup, find_packages

setup(
    name='analog_api',
    version='0.1.0',
    packages=find_packages(),
    install_requires=['requests>=2.0.0'],  # Add any dependencies here
    author='Your Name',
    author_email='your.email@example.com',
    description='A Python API for communicating with a Lucidac API.',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    zip_safe=False,
)