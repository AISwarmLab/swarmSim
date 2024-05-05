from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize(["_GoTo.py","_Land.py","_StartTrajectory.py", "_Takeoff.py", "_UploadTrajectory.py"]))