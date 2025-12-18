from setuptools import setup, find_packages

with open("README.md", "r", encoding = "utf-8") as fh:
    long_description = fh.read()

setup(
    name='ftservo-python-sdk',
    version='2.0.0',
    packages=find_packages(),
    url='https://github.com/ftservo/FTServo_Python',
    author='ftservo',
    author_email='117692731@qq.com',
    description='This is source code from official feetech repository',
    long_description_content_type="text/markdown",
    long_description=long_description,
    install_requires=['pyserial'],
    include_package_data=True,
    classifiers=[
        "Programming Language :: Python :: 3",
        'License :: OSI Approved :: MIT License',
        "Operating System :: OS Independent",
        'Topic :: System :: Hardware :: Universal Serial Bus (USB) :: Communications Device Class (CDC)'
    ]
)
