import sys

from pybind11 import get_cmake_dir

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.2.1"

ext_modules = [
    Pybind11Extension("PyS4DCEL",
        ["src/module_main.cpp"],
        # Example: passing in the version to the compiled code
        define_macros = [('VERSION_INFO', __version__)],
        ),
]

setup(
    name="PyS4DCEL",
    version=__version__,
    author="",
    author_email="",
    url="https://github.com/R-15-Ricardo/S4DCELpp",
    description="DCEL Python module on cpp ",
    long_description="",
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.6",
)
