from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['manual_mode'],
	package_dir={'': 'lib'}
)

setup(**setup_args)