# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yons/projects/PointCloudRegistrationTool

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yons/projects/PointCloudRegistrationTool

# Include any dependencies generated for this target.
include extern/gflags/CMakeFiles/gflags_nothreads_static.dir/depend.make

# Include the progress variables for this target.
include extern/gflags/CMakeFiles/gflags_nothreads_static.dir/progress.make

# Include the compile flags for this target's objects.
include extern/gflags/CMakeFiles/gflags_nothreads_static.dir/flags.make

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/flags.make
extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o: extern/gflags/src/gflags.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/PointCloudRegistrationTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o -c /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags.cc

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.i"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags.cc > CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.i

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.s"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags.cc -o CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.s

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.requires:

.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.requires

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.provides: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.requires
	$(MAKE) -f extern/gflags/CMakeFiles/gflags_nothreads_static.dir/build.make extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.provides.build
.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.provides

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.provides.build: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o


extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/flags.make
extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o: extern/gflags/src/gflags_reporting.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/PointCloudRegistrationTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o -c /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags_reporting.cc

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.i"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags_reporting.cc > CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.i

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.s"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags_reporting.cc -o CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.s

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.requires:

.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.requires

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.provides: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.requires
	$(MAKE) -f extern/gflags/CMakeFiles/gflags_nothreads_static.dir/build.make extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.provides.build
.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.provides

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.provides.build: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o


extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/flags.make
extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o: extern/gflags/src/gflags_completions.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/PointCloudRegistrationTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o -c /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags_completions.cc

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.i"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags_completions.cc > CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.i

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.s"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/PointCloudRegistrationTool/extern/gflags/src/gflags_completions.cc -o CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.s

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.requires:

.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.requires

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.provides: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.requires
	$(MAKE) -f extern/gflags/CMakeFiles/gflags_nothreads_static.dir/build.make extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.provides.build
.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.provides

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.provides.build: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o


# Object files for target gflags_nothreads_static
gflags_nothreads_static_OBJECTS = \
"CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o" \
"CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o" \
"CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o"

# External object files for target gflags_nothreads_static
gflags_nothreads_static_EXTERNAL_OBJECTS =

extern/gflags/libgflags_nothreads.a: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o
extern/gflags/libgflags_nothreads.a: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o
extern/gflags/libgflags_nothreads.a: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o
extern/gflags/libgflags_nothreads.a: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/build.make
extern/gflags/libgflags_nothreads.a: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yons/projects/PointCloudRegistrationTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libgflags_nothreads.a"
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && $(CMAKE_COMMAND) -P CMakeFiles/gflags_nothreads_static.dir/cmake_clean_target.cmake
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gflags_nothreads_static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
extern/gflags/CMakeFiles/gflags_nothreads_static.dir/build: extern/gflags/libgflags_nothreads.a

.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/build

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/requires: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags.cc.o.requires
extern/gflags/CMakeFiles/gflags_nothreads_static.dir/requires: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_reporting.cc.o.requires
extern/gflags/CMakeFiles/gflags_nothreads_static.dir/requires: extern/gflags/CMakeFiles/gflags_nothreads_static.dir/src/gflags_completions.cc.o.requires

.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/requires

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/clean:
	cd /home/yons/projects/PointCloudRegistrationTool/extern/gflags && $(CMAKE_COMMAND) -P CMakeFiles/gflags_nothreads_static.dir/cmake_clean.cmake
.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/clean

extern/gflags/CMakeFiles/gflags_nothreads_static.dir/depend:
	cd /home/yons/projects/PointCloudRegistrationTool && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yons/projects/PointCloudRegistrationTool /home/yons/projects/PointCloudRegistrationTool/extern/gflags /home/yons/projects/PointCloudRegistrationTool /home/yons/projects/PointCloudRegistrationTool/extern/gflags /home/yons/projects/PointCloudRegistrationTool/extern/gflags/CMakeFiles/gflags_nothreads_static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : extern/gflags/CMakeFiles/gflags_nothreads_static.dir/depend

