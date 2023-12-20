## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['irsl_choreonoid_ros'],
    package_dir={'irsl_choreonoid_ros': 'irsl_choreonoid_ros'},
)

setup(**setup_args)
