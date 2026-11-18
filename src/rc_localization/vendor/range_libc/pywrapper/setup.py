"""Builds the RangeLibc Cython extension; the command is in the README's Install section.

RangeLibc is Apache-2.0, by Corey H. Walsh (see ../license.txt), vendored from
f1tenth/range_libc and trimmed of the demo and benchmark files this car never
uses. The library is unchanged. This build file is rewritten for this
repository: CUDA for the Jetson Orin's GPU, and the chunk sizes kernels.cu needs.
"""
import os
from os.path import join as pjoin

import numpy
from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils.old_build_ext import old_build_ext as build_ext

use_cuda = os.environ.get("WITH_CUDA", "").lower() == "on"

compiler_flags = ["-w", "-std=c++11", "-march=native", "-ffast-math", "-fno-math-errno", "-O3"]
include_dirs = ["../", numpy.get_include()]
sources = ["RangeLibc.pyx", "../vendor/lodepng/lodepng.cpp"]


def locate_cuda() -> dict:
    """Finds the CUDA toolchain via $CUDAHOME, or common install paths."""
    for home in (os.environ.get("CUDAHOME"), "/usr/local/cuda", "/usr/local/cuda-10.2"):
        if home and os.path.isdir(home):
            return {
                "home": home,
                "nvcc": pjoin(home, "bin", "nvcc"),
                "include": pjoin(home, "include"),
                "lib64": pjoin(home, "lib64"),
            }
    raise EnvironmentError("CUDA requested (WITH_CUDA=ON) but no install found - set $CUDAHOME")


def customize_compiler_for_nvcc(compiler) -> None:
    """Dispatches .cu sources to nvcc, everything else to the default compiler."""
    compiler.src_extensions.append(".cu")
    default_compiler_so = compiler.compiler_so
    default_compile = compiler._compile

    def _compile(obj, src, ext, cc_args, extra_postargs, pp_opts):
        if os.path.splitext(src)[1] == ".cu":
            compiler.set_executable("compiler_so", cuda["nvcc"])
            extra_postargs = extra_postargs["nvcc"]
        else:
            extra_postargs = extra_postargs["gcc"]
        default_compile(obj, src, ext, cc_args, extra_postargs, pp_opts)
        compiler.compiler_so = default_compiler_so

    compiler._compile = _compile


class CudaAwareBuildExt(build_ext):
    def build_extensions(self):
        customize_compiler_for_nvcc(self.compiler)
        build_ext.build_extensions(self)


if use_cuda:
    cuda = locate_cuda()
    # kernels.cu reads CHUNK_SIZE and NUM_THREADS but includes only
    # CudaRangeLib.h, which defines neither: upstream sets them in RangeLib.h
    # (and spells the second one CHUNK_THREADS), a header the .cu never sees.
    # Passing them to both compilers keeps the device buffers kernels.cu
    # allocates the same size as the chunks RangeLib.h feeds them.
    chunking = ["-DCHUNK_SIZE=262144", "-DNUM_THREADS=256"]
    nvcc_flags = ["-arch=sm_87", "-c", "--compiler-options", "'-fPIC'", "-w", "-std=c++11"]
    ext = Extension(
        "range_libc",
        sources + ["kernels.cu"],
        extra_compile_args={
            "gcc": compiler_flags + ["-DUSE_CUDA=1"] + chunking,
            "nvcc": nvcc_flags + ["-DUSE_CUDA=1"] + chunking,
        },
        extra_link_args=["-std=c++11"],
        include_dirs=include_dirs + [cuda["include"]],
        library_dirs=[cuda["lib64"]],
        libraries=["cudart"],
        runtime_library_dirs=[cuda["lib64"]],
        depends=["../includes/*.h"],
        language="c++",
    )
    cmdclass = {"build_ext": CudaAwareBuildExt}
else:
    ext = Extension(
        "range_libc",
        sources,
        extra_compile_args=compiler_flags,
        extra_link_args=["-std=c++11"],
        include_dirs=include_dirs,
        depends=["../includes/*.h"],
        language="c++",
    )
    cmdclass = {"build_ext": build_ext}

setup(name="range_libc", version="0.1", author="Corey Walsh", ext_modules=[ext], cmdclass=cmdclass)
