import pathlib
from setuptools import setup

# The directory containing this file
HERE = pathlib.Path(__file__).parent

# The text of the README file
README = (HERE / "README.md").read_text()

# This call to setup() does all the work
setup(
    name="emachinery",
    version="1.0.0",
    description="A package for electric machinery analysis.",
    long_description=README,
    long_description_content_type="text/markdown",
    url="https://github.com/horychen/emachinery",
    author="Jiahao Chen",
    author_email="horychen@qq.com",
    license="GNU General Public License v3.0",
    classifiers=[
        "License :: OSI Approved :: GNU General Public License v3.0 License",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    packages=["core"],
    include_package_data=True,
    install_requires=[],
    entry_points={
        "console_scripts": [
            "emy=core.__main__:main",
        ]
    },
)
