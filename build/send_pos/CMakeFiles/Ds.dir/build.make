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
CMAKE_SOURCE_DIR = /home/bonato/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bonato/catkin_ws/build

# Include any dependencies generated for this target.
include send_pos/CMakeFiles/Ds.dir/depend.make

# Include the progress variables for this target.
include send_pos/CMakeFiles/Ds.dir/progress.make

# Include the compile flags for this target's objects.
include send_pos/CMakeFiles/Ds.dir/flags.make

send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o: send_pos/CMakeFiles/Ds.dir/flags.make
send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o: /home/bonato/catkin_ws/src/send_pos/src/Ds.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bonato/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o"
	cd /home/bonato/catkin_ws/build/send_pos && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Ds.dir/src/Ds.cpp.o -c /home/bonato/catkin_ws/src/send_pos/src/Ds.cpp

send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Ds.dir/src/Ds.cpp.i"
	cd /home/bonato/catkin_ws/build/send_pos && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bonato/catkin_ws/src/send_pos/src/Ds.cpp > CMakeFiles/Ds.dir/src/Ds.cpp.i

send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Ds.dir/src/Ds.cpp.s"
	cd /home/bonato/catkin_ws/build/send_pos && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bonato/catkin_ws/src/send_pos/src/Ds.cpp -o CMakeFiles/Ds.dir/src/Ds.cpp.s

send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.requires:

.PHONY : send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.requires

send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.provides: send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.requires
	$(MAKE) -f send_pos/CMakeFiles/Ds.dir/build.make send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.provides.build
.PHONY : send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.provides

send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.provides.build: send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o


# Object files for target Ds
Ds_OBJECTS = \
"CMakeFiles/Ds.dir/src/Ds.cpp.o"

# External object files for target Ds
Ds_EXTERNAL_OBJECTS =

/home/bonato/catkin_ws/devel/lib/send_pos/Ds: send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: send_pos/CMakeFiles/Ds.dir/build.make
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /home/bonato/catkin_ws/devel/lib/libiiwa_tools.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /home/bonato/catkin_ws/devel/lib/libtrac_ik.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libnlopt.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libm.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libkdl_parser.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/liburdf.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libclass_loader.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/libPocoFoundation.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libdl.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libroslib.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/librospack.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/librosconsole_bridge.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libroscpp.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/librosconsole.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/librostime.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /opt/ros/melodic/lib/libcpp_common.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/local/lib/libmc_rbdyn_urdf.so.1.1.0
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/local/lib/libRBDyn.so.1.7.2
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/bonato/catkin_ws/devel/lib/send_pos/Ds: send_pos/CMakeFiles/Ds.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bonato/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/bonato/catkin_ws/devel/lib/send_pos/Ds"
	cd /home/bonato/catkin_ws/build/send_pos && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Ds.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
send_pos/CMakeFiles/Ds.dir/build: /home/bonato/catkin_ws/devel/lib/send_pos/Ds

.PHONY : send_pos/CMakeFiles/Ds.dir/build

send_pos/CMakeFiles/Ds.dir/requires: send_pos/CMakeFiles/Ds.dir/src/Ds.cpp.o.requires

.PHONY : send_pos/CMakeFiles/Ds.dir/requires

send_pos/CMakeFiles/Ds.dir/clean:
	cd /home/bonato/catkin_ws/build/send_pos && $(CMAKE_COMMAND) -P CMakeFiles/Ds.dir/cmake_clean.cmake
.PHONY : send_pos/CMakeFiles/Ds.dir/clean

send_pos/CMakeFiles/Ds.dir/depend:
	cd /home/bonato/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bonato/catkin_ws/src /home/bonato/catkin_ws/src/send_pos /home/bonato/catkin_ws/build /home/bonato/catkin_ws/build/send_pos /home/bonato/catkin_ws/build/send_pos/CMakeFiles/Ds.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : send_pos/CMakeFiles/Ds.dir/depend

