from distutils.core import setup
from Cython.Build import cythonize  

setup(name='libs',
      ext_modules=cythonize(["libs/sum.py", "libs/__init__.py", "fooClass.pyx"],
                            compiler_directives={'language_level': '3'}))
