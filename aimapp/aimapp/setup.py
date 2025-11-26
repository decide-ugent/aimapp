import setuptools

with open("requirements.txt", "r") as req:
    requirements = req.read().splitlines()

setuptools.setup(
    name="higher_level_nav_warehouse",
    version="0.1.5",
    author="DML group",
    author_email="",
    description=" D navigation at the highest level of abstraction adapted for ros2 and real observations",
    long_description="Check Readme",
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
        "Development Status :: 4 - Beta"
    ],
    python_requires=">=3.8",
    install_requires=requirements,

)