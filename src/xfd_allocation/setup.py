from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(
    version='0.0.0',
    scripts=['scripts/test.py','scripts/mrta_server.py'],
    packages=['xfd_allocation'],
    package_dir={'': 'scripts'}
)