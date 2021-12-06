# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
# https://zhuanlan.zhihu.com/p/58382081
setup_args = generate_distutils_setup(
    packages=['segmentor'],
    package_dir={'': 'include/sensor/camera'})

setup(**setup_args)
