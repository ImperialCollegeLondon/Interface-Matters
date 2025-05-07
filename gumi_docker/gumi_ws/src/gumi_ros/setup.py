from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['gumi_ros'],
    package_dir={'': 'scipts'}
)

setup(**setup_args)
