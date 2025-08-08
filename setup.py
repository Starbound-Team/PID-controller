from setuptools import setup, find_packages

setup(
    name="drone-pid-controller",
    version="0.1.1",
    author="AlexBesios",
    author_email="alexbesios@gmail.com",
    description="Robust multi-axis PID control framework with tuning & visualization for VTOL / multirotor UAVs",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/Starbound-Team/PID-controller",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy",
        "pyyaml",
    ],
    extras_require={
        "tuning": ["scipy", "matplotlib"],
        "hardware": [
            "RPi.GPIO; platform_system=='Linux'",
            "fake-rpi; platform_system!='Linux'",
            "smbus2; platform_system=='Linux'",
        ],
        "dev": ["pytest", "scipy", "matplotlib"],
        "all": [
            "scipy",
            "matplotlib",
            "RPi.GPIO; platform_system=='Linux'",
            "fake-rpi; platform_system!='Linux'",
            "smbus2; platform_system=='Linux'",
            "pytest",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
