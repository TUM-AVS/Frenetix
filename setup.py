import os
import glob
import shutil

from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension, build_ext, ParallelCompile, naive_recompile

# get the current directory
current_dir = os.path.dirname(os.path.realpath(__file__))
third_party_dir = os.path.join(current_dir, "thirdParty")
# Define build directory
third_party_lib_dir = os.path.join(third_party_dir, "driveabilityChecker", "lib64")
build_dir = os.path.join(current_dir, "build")

class CustomBuildExt(build_ext):
    def run(self):
        print("CustomBuildExt.run() called")
        # First, run the original build_ext command
        build_ext.run(self)
        
        lib_dir = os.path.join(self.build_lib, "frenetPlannerHelper")
        if not os.path.exists(lib_dir):
            os.makedirs(lib_dir)
           
        # Copy the third-party shared libraries
        
        for _file in glob.glob(os.path.join(third_party_lib_dir, "*.so")):
            print("copying ", _file, " to ", os.path.join(lib_dir, os.path.basename(_file)))
            shutil.copy2(_file, os.path.join(lib_dir, os.path.basename(_file)))




# Check if build directory exists, if so, remove it
if os.path.exists(build_dir):
    shutil.rmtree(build_dir)

src_dir = os.path.join(current_dir, "src")

# get all the .cpp files in the src directory
cpp_files = glob.glob(os.path.join(src_dir, "**", '*.cpp'), recursive=True)

# define the extension module
ext_modules = [
    Pybind11Extension(
        "frenetPlannerHelper",
        cpp_files,
        include_dirs=
        [
            os.path.join(third_party_dir,"eigen"),
            os.path.join(third_party_dir,"taskflow"),
            os.path.join(third_party_dir,"driveabilityChecker", "include"),
            src_dir,
            os.path.join(src_dir, "trajectory"),
            os.path.join(src_dir, "strategies")
        ],
        libraries=
        [
            "crccosy",
            "s11n"
        ],
        library_dirs=
        [
            third_party_lib_dir,
        ],
        extra_compile_args=
        [
            "-g",
            "-fopenmp",
            "-Wno-unused-function",
            "-std=c++17",
            "-static-libstdc++",
            #"-O0",  # turn off optimization
        ],
        extra_link_args=
        [
            "-fopenmp",
            r"-Wl,-rpath,$ORIGIN/frenetPlannerHelper",
        ],
    ),
]

ParallelCompile("NPY_NUM_BUILD_JOBS", needs_recompile=naive_recompile).install()

setup(
    name="frenetPlannerHelper",
    package_data=
    {
        'frenetPlannerHelper': ['*.so'],
    },
    version="1.0.0",
    cmdclass={"build_ext": CustomBuildExt},
    ext_modules=ext_modules,
)
