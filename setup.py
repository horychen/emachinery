import pathlib
from setuptools import setup, find_packages

# The directory containing this file
HERE = pathlib.Path(__file__).parent

# The text of the README file
README = (HERE / "README.md").read_text(encoding='utf-8')
CHANGELOG = (HERE / "CHANGELOG.txt").read_text(encoding='utf-8')

buf = (HERE / "emachinery/__init__.py").read_text(encoding='utf-8')
loc1 = buf.find('__version__') + len('__version__')
loc2 = buf[loc1:].find("'") + 1
loc3 = buf[loc1+loc2:].find("'")
VERSION = buf[loc1+loc2:loc1+loc2+loc3]
print('CJH AUTO DETECT VERSION:', VERSION)

# This call to setup() does all the work
setup(
    name="emachinery",
    version=VERSION,
    description="A package for electric machinery analysis.",
    long_description=README + '\n\n' + CHANGELOG,
    long_description_content_type="text/markdown",
    url="https://github.com/horychen/emachinery",
    author="Jiahao Chen",
    author_email="horychen@qq.com",
    license="GNU General Public License v3.0",
    classifiers=[ # see https://pypi.org/classifiers/
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    keywords='electric machinery, motors, motor control', 
    # package_dir={'emachinery':'core'},
    # packages=find_packages(),
    packages=["emachinery"],
    include_package_data=True,
    install_requires=[],
    entry_points={
        "console_scripts": [
            "emy=emachinery.__main__:main",
        ]
    },
)
