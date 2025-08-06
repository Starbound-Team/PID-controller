from setuptools import setup, find_packages

setup(
    name="drone-pid-controller",
    version="0.1.0",
    author="AlexBesios",
    author_email="alexbesios@gmail.com",
    description="A PID controller implementation for multirotor UAVs",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/Starbound-Team/PID-controller",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy",
        "scipy", 
        "matplotlib",
        "pyyaml",
        "pytest",
        "RPi.GPIO; platform_system=='Linux'",
        "fake-rpi; platform_system!='Linux'",
        "smbus2; platform_system=='Linux'",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
