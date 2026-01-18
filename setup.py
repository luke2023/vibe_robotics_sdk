from setuptools import setup, find_packages

setup(
    name='viberobotics',
    version='0.1.0',    
    description='Python package for Vibe Robotics',
    url='https://github.com/viberobotics/viberobotics-python',
    author='Vibe Robotics',
    author_email='',
    license='Apache 2.0',
    packages=find_packages(),
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=[],
)