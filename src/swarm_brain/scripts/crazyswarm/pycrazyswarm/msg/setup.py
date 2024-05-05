from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize(["_FullState.py","_GenericLogData.py","_Position.py", "_TrajectoryPolynomialPiece.py", "_VelocityWorld.py"]))