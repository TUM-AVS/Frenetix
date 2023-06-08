import sys

try:
    from skbuild import setup
except ImportError:
    print('Please update pip, you need pip 10 or greater,\n'
          ' or you need to install the PEP 518 requirements in pyproject.toml yourself', file=sys.stderr)
    raise

from setuptools import find_packages

setup(
    packages=find_packages(where="src/python"),
    package_dir={"": "src/python"},
    include_package_data=False,

    cmake_install_dir='src/python/frenetPlannerHelper',
    cmake_install_target='install-python-modules-frenetPlannerHelper',
)
