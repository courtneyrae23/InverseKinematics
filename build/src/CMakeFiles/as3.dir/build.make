# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/cc/cs184/fa16/class/cs184-aai/as/as4_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build

# Include any dependencies generated for this target.
include src/CMakeFiles/as3.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/as3.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/as3.dir/flags.make

src/CMakeFiles/as3.dir/example_03.cpp.o: src/CMakeFiles/as3.dir/flags.make
src/CMakeFiles/as3.dir/example_03.cpp.o: ../src/example_03.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/as3.dir/example_03.cpp.o"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/as3.dir/example_03.cpp.o -c /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src/example_03.cpp

src/CMakeFiles/as3.dir/example_03.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/as3.dir/example_03.cpp.i"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src/example_03.cpp > CMakeFiles/as3.dir/example_03.cpp.i

src/CMakeFiles/as3.dir/example_03.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/as3.dir/example_03.cpp.s"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src/example_03.cpp -o CMakeFiles/as3.dir/example_03.cpp.s

src/CMakeFiles/as3.dir/example_03.cpp.o.requires:
.PHONY : src/CMakeFiles/as3.dir/example_03.cpp.o.requires

src/CMakeFiles/as3.dir/example_03.cpp.o.provides: src/CMakeFiles/as3.dir/example_03.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/as3.dir/build.make src/CMakeFiles/as3.dir/example_03.cpp.o.provides.build
.PHONY : src/CMakeFiles/as3.dir/example_03.cpp.o.provides

src/CMakeFiles/as3.dir/example_03.cpp.o.provides.build: src/CMakeFiles/as3.dir/example_03.cpp.o

src/CMakeFiles/as3.dir/data_structures.cpp.o: src/CMakeFiles/as3.dir/flags.make
src/CMakeFiles/as3.dir/data_structures.cpp.o: ../src/data_structures.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/as3.dir/data_structures.cpp.o"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/as3.dir/data_structures.cpp.o -c /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src/data_structures.cpp

src/CMakeFiles/as3.dir/data_structures.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/as3.dir/data_structures.cpp.i"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src/data_structures.cpp > CMakeFiles/as3.dir/data_structures.cpp.i

src/CMakeFiles/as3.dir/data_structures.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/as3.dir/data_structures.cpp.s"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src/data_structures.cpp -o CMakeFiles/as3.dir/data_structures.cpp.s

src/CMakeFiles/as3.dir/data_structures.cpp.o.requires:
.PHONY : src/CMakeFiles/as3.dir/data_structures.cpp.o.requires

src/CMakeFiles/as3.dir/data_structures.cpp.o.provides: src/CMakeFiles/as3.dir/data_structures.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/as3.dir/build.make src/CMakeFiles/as3.dir/data_structures.cpp.o.provides.build
.PHONY : src/CMakeFiles/as3.dir/data_structures.cpp.o.provides

src/CMakeFiles/as3.dir/data_structures.cpp.o.provides.build: src/CMakeFiles/as3.dir/data_structures.cpp.o

# Object files for target as3
as3_OBJECTS = \
"CMakeFiles/as3.dir/example_03.cpp.o" \
"CMakeFiles/as3.dir/data_structures.cpp.o"

# External object files for target as3
as3_EXTERNAL_OBJECTS =

as3: src/CMakeFiles/as3.dir/example_03.cpp.o
as3: src/CMakeFiles/as3.dir/data_structures.cpp.o
as3: src/CMakeFiles/as3.dir/build.make
as3: glew/libglew.a
as3: glew/libglew.a
as3: glfw-3.2.1/src/libglfw3.a
as3: /usr/lib/x86_64-linux-gnu/libGLU.so
as3: /usr/lib/x86_64-linux-gnu/libGL.so
as3: /usr/lib/x86_64-linux-gnu/libSM.so
as3: /usr/lib/x86_64-linux-gnu/libICE.so
as3: /usr/lib/x86_64-linux-gnu/libX11.so
as3: /usr/lib/x86_64-linux-gnu/libXext.so
as3: /usr/lib/x86_64-linux-gnu/librt.so
as3: /usr/lib/x86_64-linux-gnu/libm.so
as3: /usr/lib/x86_64-linux-gnu/libXrandr.so
as3: /usr/lib/x86_64-linux-gnu/libXinerama.so
as3: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
as3: /usr/lib/x86_64-linux-gnu/libXcursor.so
as3: /usr/lib/x86_64-linux-gnu/libX11.so
as3: src/CMakeFiles/as3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../as3"
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/as3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/as3.dir/build: as3
.PHONY : src/CMakeFiles/as3.dir/build

src/CMakeFiles/as3.dir/requires: src/CMakeFiles/as3.dir/example_03.cpp.o.requires
src/CMakeFiles/as3.dir/requires: src/CMakeFiles/as3.dir/data_structures.cpp.o.requires
.PHONY : src/CMakeFiles/as3.dir/requires

src/CMakeFiles/as3.dir/clean:
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src && $(CMAKE_COMMAND) -P CMakeFiles/as3.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/as3.dir/clean

src/CMakeFiles/as3.dir/depend:
	cd /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cc/cs184/fa16/class/cs184-aai/as/as4_2 /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/src /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src /home/cc/cs184/fa16/class/cs184-aai/as/as4_2/build/src/CMakeFiles/as3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/as3.dir/depend

