from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="drone-pid-controller",
    version="0.1.1",
    author="AlexBesios",
    author_email="alexbesios@gmail.com",
    description="Robust multi-axis PID control framework with tuning & visualization for VTOL / multirotor UAVs",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Starbound-Team/PID-controller",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy>=1.19.0",
        "pyyaml>=5.4.0",
    ],
    extras_require={
        "tuning": ["scipy>=1.7.0", "matplotlib>=3.3.0"],
        "hardware": [
            "RPi.GPIO>=0.7.0; platform_system=='Linux'",
            "fake-rpi>=0.7.0; platform_system!='Linux'",
            "smbus2>=0.4.0; platform_system=='Linux'",
            "pyserial>=3.5",
        ],
        "dev": ["pytest>=6.0.0", "scipy>=1.7.0", "matplotlib>=3.3.0"],
        "all": [
            "scipy>=1.7.0",
            "matplotlib>=3.3.0",
            "RPi.GPIO>=0.7.0; platform_system=='Linux'",
            "fake-rpi>=0.7.0; platform_system!='Linux'",
            "smbus2>=0.4.0; platform_system=='Linux'",
            "pyserial>=3.5",
            "pytest>=6.0.0",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
        "Operating System :: OS Independent",
    ],
    license="MIT",
    python_requires=">=3.8",
    keywords="pid controller drone uav vtol control systems robotics autopilot",
    project_urls={
        "Bug Reports": "https://github.com/Starbound-Team/PID-controller/issues",
        "Source": "https://github.com/Starbound-Team/PID-controller",
        "Documentation": "https://github.com/Starbound-Team/PID-controller#readme",
    },
)
