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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/malalves/gazebo_plugin_tutorial

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/malalves/gazebo_plugin_tutorial/build

# Include any dependencies generated for this target.
include CMakeFiles/teste.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/teste.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/teste.dir/flags.make

CMakeFiles/teste.dir/teste.cc.o: CMakeFiles/teste.dir/flags.make
CMakeFiles/teste.dir/teste.cc.o: ../teste.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/malalves/gazebo_plugin_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/teste.dir/teste.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/teste.dir/teste.cc.o -c /home/malalves/gazebo_plugin_tutorial/teste.cc

CMakeFiles/teste.dir/teste.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/teste.dir/teste.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/malalves/gazebo_plugin_tutorial/teste.cc > CMakeFiles/teste.dir/teste.cc.i

CMakeFiles/teste.dir/teste.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/teste.dir/teste.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/malalves/gazebo_plugin_tutorial/teste.cc -o CMakeFiles/teste.dir/teste.cc.s

CMakeFiles/teste.dir/teste.cc.o.requires:

.PHONY : CMakeFiles/teste.dir/teste.cc.o.requires

CMakeFiles/teste.dir/teste.cc.o.provides: CMakeFiles/teste.dir/teste.cc.o.requires
	$(MAKE) -f CMakeFiles/teste.dir/build.make CMakeFiles/teste.dir/teste.cc.o.provides.build
.PHONY : CMakeFiles/teste.dir/teste.cc.o.provides

CMakeFiles/teste.dir/teste.cc.o.provides.build: CMakeFiles/teste.dir/teste.cc.o


# Object files for target teste
teste_OBJECTS = \
"CMakeFiles/teste.dir/teste.cc.o"

# External object files for target teste
teste_EXTERNAL_OBJECTS =

teste: CMakeFiles/teste.dir/teste.cc.o
teste: CMakeFiles/teste.dir/build.make
teste: /usr/lib/x86_64-linux-gnu/libgazebo.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
teste: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
teste: /usr/lib/x86_64-linux-gnu/libboost_thread.so
teste: /usr/lib/x86_64-linux-gnu/libboost_signals.so
teste: /usr/lib/x86_64-linux-gnu/libboost_system.so
teste: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
teste: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
teste: /usr/lib/x86_64-linux-gnu/libboost_regex.so
teste: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
teste: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
teste: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
teste: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
teste: /usr/lib/x86_64-linux-gnu/libpthread.so
teste: /usr/lib/x86_64-linux-gnu/libprotobuf.so
teste: /usr/lib/x86_64-linux-gnu/libsdformat.so
teste: /usr/lib/x86_64-linux-gnu/libignition-math2.so
teste: /usr/lib/x86_64-linux-gnu/libOgreMain.so
teste: /usr/lib/x86_64-linux-gnu/libboost_thread.so
teste: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
teste: /usr/lib/x86_64-linux-gnu/libboost_system.so
teste: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
teste: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
teste: /usr/lib/x86_64-linux-gnu/libpthread.so
teste: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
teste: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
teste: /usr/lib/x86_64-linux-gnu/libignition-math2.so
teste: /usr/lib/x86_64-linux-gnu/libboost_thread.so
teste: /usr/lib/x86_64-linux-gnu/libboost_signals.so
teste: /usr/lib/x86_64-linux-gnu/libboost_system.so
teste: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
teste: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
teste: /usr/lib/x86_64-linux-gnu/libboost_regex.so
teste: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
teste: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
teste: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
teste: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
teste: /usr/lib/x86_64-linux-gnu/libpthread.so
teste: /usr/lib/x86_64-linux-gnu/libprotobuf.so
teste: /usr/lib/x86_64-linux-gnu/libsdformat.so
teste: /usr/lib/x86_64-linux-gnu/libOgreMain.so
teste: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
teste: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
teste: CMakeFiles/teste.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/malalves/gazebo_plugin_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable teste"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/teste.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/teste.dir/build: teste

.PHONY : CMakeFiles/teste.dir/build

CMakeFiles/teste.dir/requires: CMakeFiles/teste.dir/teste.cc.o.requires

.PHONY : CMakeFiles/teste.dir/requires

CMakeFiles/teste.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/teste.dir/cmake_clean.cmake
.PHONY : CMakeFiles/teste.dir/clean

CMakeFiles/teste.dir/depend:
	cd /home/malalves/gazebo_plugin_tutorial/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/malalves/gazebo_plugin_tutorial /home/malalves/gazebo_plugin_tutorial /home/malalves/gazebo_plugin_tutorial/build /home/malalves/gazebo_plugin_tutorial/build /home/malalves/gazebo_plugin_tutorial/build/CMakeFiles/teste.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/teste.dir/depend
