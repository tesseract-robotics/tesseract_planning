# Tesseract Planning

[![codecov](https://codecov.io/gh/tesseract-robotics/tesseract_planning/branch/master/graph/badge.svg)](https://codecov.io/gh/tesseract-robotics/tesseract_planning)

[![Python](https://img.shields.io/badge/python-2.7+|3.6+-blue.svg)](https://github.com/tesseract-robotics/tesseract_python)

Platform             | CI Status
---------------------|:---------
Linux (Stable)        | [![Ubuntu](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/ubuntu.yml)
Linux (Unstable)     | [![Unstable](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/unstable.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/unstable.yml)
Windows              | [![Windows-Noetic-Build](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/windows.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/windows.yml)
Lint  (Clang-Format) | [![Clang-Format](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/clang_format.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/clang_format.yml)
Lint  (CMake-Format) | [![CMake-Format](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/cmake_format.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/cmake_format.yml)
Lint  (Clang-Tidy)   | [![Code Quality](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/code_quality.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/code_quality.yml)
Lint  (CodeCov)      | [![Code Quality](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/code_quality.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_planning/actions/workflows/code_quality.yml)

[![Github Issues](https://img.shields.io/github/issues/tesseract-robotics/tesseract_planning.svg)](http://github.com/tesseract-robotics/tesseract_planning/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

This include packages related to both motion and process planning for the Tesseract Motion Planning Environment.

## Tesseract Setup Wizard and Visualization Tools

[![](https://github.com/snapcore/snap-store-badges/blob/master/EN/%5BEN%5D-snap-store-black-uneditable%401x.png)](https://snapcraft.io/tesseract-ignition)

## Tesseract Planning Packages

* **tesseract_motion_planners** – This package contains a common interface for Planners and includes implementation for OMPL, TrajOpt, TrajOpt IFOPT and Descartes.
* **tesseract_process_managers** – This package contains a common interface for Process Planning and includes implementation for a wide variaty of process found industrial automation like painting, griding, welding, pick and place and more.
* **tesseract_time_parameterization** – This package contains a time parameterization algorithms and includes iteritive spline.

## Related Repositories

* [Tesseract](https://github.com/tesseract-robotics/tesseract)
* [Tesseract Python](https://github.com/tesseract-robotics/tesseract_python)
* [Tesseract ROS](https://github.com/tesseract-robotics/tesseract_ros)
* [Tesseract Documentation](https://github.com/tesseract-robotics/tesseract_docs)

## Documentation

* [Wiki](https://tesseract-docs.readthedocs.io)
* [Doxygen](https://tesseract-robotics.github.io/tesseract_planning/)
* [Benchmark](https://tesseract-robotics.github.io/tesseract_planning/dev/bench)

## Process Planning Server Tuning
The planning server is built leveraging the library taskflow which enables you to quickly write parallel and heterogeneous tasks programs which aligns well with process motion planning. These types of libraries leverage thread pools to quickly spin up different tasks as needed. One interesting thing found, if a task consumes a lot of memory, then the malloc library can make a performance decision to allow the thread to hold onto the memory it has for subsequent tasks opposed to giving the memory back to your system. This may be fine if this was the only set of tasks being run but typically in our use case there are other computationally intensive processes which could use this memory.

### Ubuntu

### tcmalloc (Recommend)

The recommended approach to address memory management is to leverage the `tcmalloc` library which is part of Google Performance Tools (`gperftools`). Experimentation has shown `tcmalloc` to provide more stable memory management when leveraging the planning server.

This capability can be leveraged by linking the exectuable that contains the planning server against `tcmalloc` using the following commands:

``` cmake
find_package(tesseract_common REQUIRED)
find_package(tcmalloc REQUIRED)

add_executable(my_executable src/my_executable.cpp)
target_link_libraries(my_executable PUBLIC tcmalloc::tcmalloc)
```

This library provides a set of run-time [tunables](https://gperftools.github.io/gperftools/tcmalloc.html), leveraging a set of environment variables.

The one parameter that has been used to configure tcmalloc to give memory back to the OS is `TCMALLOC_RELEASE_RATE`:

```
TCMALLOC_RELEASE_RATE 	default: 1.0 	Rate at which we release unused memory to the system, via madvise(MADV_DONTNEED), on systems that support it. Zero means we never release memory back to the system. Increase this flag to return memory faster; decrease it to return memory slower. Reasonable rates are in the range [0,10].
```

Note: This is typically set in the roslaunch file for the planning server node leveraging the `<env name="TCMALLOC_RELEASE_RATE" value="10">` tag.

### `ptmalloc` (Ubuntu OS default)

The great thing is, the default malloc library `ptmalloc` provides run-time [tunables](https://www.gnu.org/software/libc/manual/html_node/Tunables.html) through a set of environment variables.

The one parameter that has been used to configure libc so it always gives back the memory to the system is setting following memory tunnable:

```
Tunable: glibc.malloc.mmap_threshold

export GLIBC_TUNABLES=glibc.malloc.mmap_threshold=1000000

This tunable supersedes the MALLOC_MMAP_THRESHOLD_ environment variable and is identical in features.

When this tunable is set, all chunks larger than this value in bytes are allocated outside the normal heap, using the mmap system call. This way it is guaranteed that the memory for these chunks can be returned to the system on free. Note that requests smaller than this threshold might still be allocated via mmap.

If this tunable is not set, the default value is set to ‘131072’ bytes and the threshold is adjusted dynamically to suit the allocation patterns of the program. If the tunable is set, the dynamic adjustment is disabled and the value is set as static.
```

Note: This is typically set in the roslaunch file for the planning server node leveraging the `<env name="GLIBC_TUNABLES" value="glibc.malloc.mmap_threshold=1000000">` tag.

### Windows

TBD

## Evolution

[![Tesseract Evolution Video](gh_pages/_static/tesseract_evolution.png)](https://www.youtube.com/watch?v=rxlzlsSBxAY)

How to create:

* Create Video: `gource -1280x720 -seconds-per-day 0.2 --auto-skip-seconds 0.2 --disable-bloom -background d0d3d4 --hide filenames,mouse,progress -o - | ffmpeg -y -r 60 -f image2pipe -vcodec ppm -i - -vcodec libx264 -preset ultrafast -pix_fmt yuv420p -crf 1 -threads 0 -bf 0 gource.mp4`
* Create Gif: `ffmpeg -i gource.mp4 -r 10 -vf "scale=800:-1,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" tesseract_evolution.gif`

## TODO's

Warning: These packages are under heavy development and are subject to change.

See [issue #66](https://github.com/tesseract-robotics/tesseract/issues/66)

## Build Instructions

1) Clone repository into your workspace
2) Clone the repositories in the dependencies.repos file using [vcstool](http://wiki.ros.org/vcstool) or some other method (e.g. manually git cloning them)
3) Build the workspace using catkin tools, colcon, or a similar tool

NOTE: To speed up clean build you may want to add tesseract_ext to an extended workspace.

NOTE: Melodic install TaskFlow from [ROS-Industrial PPA](https://launchpad.net/~ros-industrial/+archive/ubuntu/ppa).

### Building with Clang-Tidy Enabled

Must pass the -DTESSERACT_ENABLE_CLANG_TIDY=ON to cmake when building. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.

### Building Tests

Must pass the -DTESSERACT_ENABLE_TESTING=ON to cmake when wanting to build tests. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.

NOTE: If you are building using catkin tools, use `catkin build --force-cmake -DTESSERACT_ENABLE_TESTING=ON`.

#### Running Tests

Tesseract packages use ctest because it is ROS agnostic, so to run the test call `catkin test --no-deps tesseract_motion_planners tesseract_process_managers tesseract_time_parameterization`

### Building Code Coverage

Must pass the -DTESSERACT_ENABLE_CODE_COVERAGE=ON to cmake when wanting to build code coverage. The code coverage report is located in each individuals build directory inside a ccov/all-merged folder. Open the index.html file to see the packages code coverage report.

NOTE: Must be a clean build when generating a code coverage report. Also must build in debug.

#### Exclude Code From Code Coverage

- LCOV_EXCL_LINE
  - Lines containing this marker will be excluded.
- LCOV_EXCL_START
  - Marks the beginning of an excluded section. The current line is part of this section.
- LCOV_EXCL_STOP
  - Marks the end of an excluded section. The current line not part of this section.

.. NOTE: You can replace **LCOV** above with **GCOV** or **GCOVR**.

## Quality Tools Leverage

Tesseract currently leverages Compiler Warnigs, Clang Tidy and Code Coverage. All warnings produced by Compiler and Clang Tidy are treated as errors during CI builds.

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.

## Debugging Windows Build

- Search Directories CI
  - dir /s /b c:\opt\ros\noetic\*assimp*
- Location of ROS Windows Builds
  - https://ros-win.visualstudio.com/ros-win/_build
  - https://ros-win.visualstudio.com/ros-win/_build/results?buildId=8711&view=artifacts&type=publishedArtifacts
