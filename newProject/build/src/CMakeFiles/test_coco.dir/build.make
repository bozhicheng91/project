# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bozhicheng/Documents/program/C++/newProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bozhicheng/Documents/program/C++/newProject/build

# Include any dependencies generated for this target.
include src/CMakeFiles/test_coco.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/test_coco.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/test_coco.dir/flags.make

src/CMakeFiles/test_coco.dir/main.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/test_coco.dir/main.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/main.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/main.cpp

src/CMakeFiles/test_coco.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/main.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/main.cpp > CMakeFiles/test_coco.dir/main.cpp.i

src/CMakeFiles/test_coco.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/main.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/main.cpp -o CMakeFiles/test_coco.dir/main.cpp.s

src/CMakeFiles/test_coco.dir/main.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/main.cpp.o.requires

src/CMakeFiles/test_coco.dir/main.cpp.o.provides: src/CMakeFiles/test_coco.dir/main.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/main.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/main.cpp.o.provides

src/CMakeFiles/test_coco.dir/main.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/main.cpp.o


src/CMakeFiles/test_coco.dir/PointCloud.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/PointCloud.cpp.o: ../src/PointCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/test_coco.dir/PointCloud.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/PointCloud.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/PointCloud.cpp

src/CMakeFiles/test_coco.dir/PointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/PointCloud.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/PointCloud.cpp > CMakeFiles/test_coco.dir/PointCloud.cpp.i

src/CMakeFiles/test_coco.dir/PointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/PointCloud.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/PointCloud.cpp -o CMakeFiles/test_coco.dir/PointCloud.cpp.s

src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.requires

src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.provides: src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.provides

src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/PointCloud.cpp.o


src/CMakeFiles/test_coco.dir/DataInput.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/DataInput.cpp.o: ../src/DataInput.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/test_coco.dir/DataInput.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/DataInput.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/DataInput.cpp

src/CMakeFiles/test_coco.dir/DataInput.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/DataInput.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/DataInput.cpp > CMakeFiles/test_coco.dir/DataInput.cpp.i

src/CMakeFiles/test_coco.dir/DataInput.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/DataInput.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/DataInput.cpp -o CMakeFiles/test_coco.dir/DataInput.cpp.s

src/CMakeFiles/test_coco.dir/DataInput.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/DataInput.cpp.o.requires

src/CMakeFiles/test_coco.dir/DataInput.cpp.o.provides: src/CMakeFiles/test_coco.dir/DataInput.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/DataInput.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/DataInput.cpp.o.provides

src/CMakeFiles/test_coco.dir/DataInput.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/DataInput.cpp.o


src/CMakeFiles/test_coco.dir/coco.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/coco.cpp.o: ../src/coco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/test_coco.dir/coco.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/coco.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/coco.cpp

src/CMakeFiles/test_coco.dir/coco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/coco.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/coco.cpp > CMakeFiles/test_coco.dir/coco.cpp.i

src/CMakeFiles/test_coco.dir/coco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/coco.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/coco.cpp -o CMakeFiles/test_coco.dir/coco.cpp.s

src/CMakeFiles/test_coco.dir/coco.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/coco.cpp.o.requires

src/CMakeFiles/test_coco.dir/coco.cpp.o.provides: src/CMakeFiles/test_coco.dir/coco.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/coco.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/coco.cpp.o.provides

src/CMakeFiles/test_coco.dir/coco.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/coco.cpp.o


src/CMakeFiles/test_coco.dir/border.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/border.cpp.o: ../src/border.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/test_coco.dir/border.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/border.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/border.cpp

src/CMakeFiles/test_coco.dir/border.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/border.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/border.cpp > CMakeFiles/test_coco.dir/border.cpp.i

src/CMakeFiles/test_coco.dir/border.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/border.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/border.cpp -o CMakeFiles/test_coco.dir/border.cpp.s

src/CMakeFiles/test_coco.dir/border.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/border.cpp.o.requires

src/CMakeFiles/test_coco.dir/border.cpp.o.provides: src/CMakeFiles/test_coco.dir/border.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/border.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/border.cpp.o.provides

src/CMakeFiles/test_coco.dir/border.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/border.cpp.o


src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o: ../src/border_facet_selection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/border_facet_selection.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/border_facet_selection.cpp

src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/border_facet_selection.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/border_facet_selection.cpp > CMakeFiles/test_coco.dir/border_facet_selection.cpp.i

src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/border_facet_selection.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/border_facet_selection.cpp -o CMakeFiles/test_coco.dir/border_facet_selection.cpp.s

src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.requires

src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.provides: src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.provides

src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o


src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o: ../src/ring_extending_coco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/ring_extending_coco.cpp

src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/ring_extending_coco.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/ring_extending_coco.cpp > CMakeFiles/test_coco.dir/ring_extending_coco.cpp.i

src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/ring_extending_coco.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/ring_extending_coco.cpp -o CMakeFiles/test_coco.dir/ring_extending_coco.cpp.s

src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.requires

src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.provides: src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.provides

src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o


src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o: ../src/test_incrementalcoco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/test_incrementalcoco.cpp

src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/test_incrementalcoco.cpp > CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.i

src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/test_incrementalcoco.cpp -o CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.s

src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.requires

src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.provides: src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.provides

src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o


src/CMakeFiles/test_coco.dir/IncLocal.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/IncLocal.cpp.o: ../src/IncLocal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/CMakeFiles/test_coco.dir/IncLocal.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/IncLocal.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/IncLocal.cpp

src/CMakeFiles/test_coco.dir/IncLocal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/IncLocal.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/IncLocal.cpp > CMakeFiles/test_coco.dir/IncLocal.cpp.i

src/CMakeFiles/test_coco.dir/IncLocal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/IncLocal.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/IncLocal.cpp -o CMakeFiles/test_coco.dir/IncLocal.cpp.s

src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.requires

src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.provides: src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.provides

src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/IncLocal.cpp.o


src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o: src/CMakeFiles/test_coco.dir/flags.make
src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o: ../src/IncTestFunc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_coco.dir/IncTestFunc.cpp.o -c /home/bozhicheng/Documents/program/C++/newProject/src/IncTestFunc.cpp

src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coco.dir/IncTestFunc.cpp.i"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bozhicheng/Documents/program/C++/newProject/src/IncTestFunc.cpp > CMakeFiles/test_coco.dir/IncTestFunc.cpp.i

src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coco.dir/IncTestFunc.cpp.s"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bozhicheng/Documents/program/C++/newProject/src/IncTestFunc.cpp -o CMakeFiles/test_coco.dir/IncTestFunc.cpp.s

src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.requires:

.PHONY : src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.requires

src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.provides: src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/test_coco.dir/build.make src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.provides.build
.PHONY : src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.provides

src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.provides.build: src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o


# Object files for target test_coco
test_coco_OBJECTS = \
"CMakeFiles/test_coco.dir/main.cpp.o" \
"CMakeFiles/test_coco.dir/PointCloud.cpp.o" \
"CMakeFiles/test_coco.dir/DataInput.cpp.o" \
"CMakeFiles/test_coco.dir/coco.cpp.o" \
"CMakeFiles/test_coco.dir/border.cpp.o" \
"CMakeFiles/test_coco.dir/border_facet_selection.cpp.o" \
"CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o" \
"CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o" \
"CMakeFiles/test_coco.dir/IncLocal.cpp.o" \
"CMakeFiles/test_coco.dir/IncTestFunc.cpp.o"

# External object files for target test_coco
test_coco_EXTERNAL_OBJECTS =

test_coco: src/CMakeFiles/test_coco.dir/main.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/PointCloud.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/DataInput.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/coco.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/border.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/IncLocal.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o
test_coco: src/CMakeFiles/test_coco.dir/build.make
test_coco: /usr/lib/x86_64-linux-gnu/libmpfr.so
test_coco: /usr/lib/x86_64-linux-gnu/libgmp.so
test_coco: /usr/local/lib/libCGAL.so.11.0.2
test_coco: /usr/lib/x86_64-linux-gnu/libboost_thread.so
test_coco: /usr/lib/x86_64-linux-gnu/libboost_system.so
test_coco: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
test_coco: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
test_coco: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
test_coco: src/CMakeFiles/test_coco.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bozhicheng/Documents/program/C++/newProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable ../test_coco"
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_coco.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/test_coco.dir/build: test_coco

.PHONY : src/CMakeFiles/test_coco.dir/build

src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/main.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/PointCloud.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/DataInput.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/coco.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/border.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/border_facet_selection.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/ring_extending_coco.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/test_incrementalcoco.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/IncLocal.cpp.o.requires
src/CMakeFiles/test_coco.dir/requires: src/CMakeFiles/test_coco.dir/IncTestFunc.cpp.o.requires

.PHONY : src/CMakeFiles/test_coco.dir/requires

src/CMakeFiles/test_coco.dir/clean:
	cd /home/bozhicheng/Documents/program/C++/newProject/build/src && $(CMAKE_COMMAND) -P CMakeFiles/test_coco.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/test_coco.dir/clean

src/CMakeFiles/test_coco.dir/depend:
	cd /home/bozhicheng/Documents/program/C++/newProject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bozhicheng/Documents/program/C++/newProject /home/bozhicheng/Documents/program/C++/newProject/src /home/bozhicheng/Documents/program/C++/newProject/build /home/bozhicheng/Documents/program/C++/newProject/build/src /home/bozhicheng/Documents/program/C++/newProject/build/src/CMakeFiles/test_coco.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/test_coco.dir/depend
