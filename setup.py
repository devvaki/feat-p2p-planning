from setuptools import setup, Extension
from Cython.Build import cythonize

exts = [
    Extension("p2p_planning.geo_cyth", ["p2p_planning/geo_cyth.pyx"]),
]

setup(
    name="p2p_planning",
    version="0.1.0",
    packages=["p2p_planning"],
    ext_modules=cythonize(
        exts,
        compiler_directives={"language_level": "3"},
    ),
)
