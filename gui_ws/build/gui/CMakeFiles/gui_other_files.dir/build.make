# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ameyasu/QTprojects/scanner-gui/gui_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ameyasu/QTprojects/scanner-gui/gui_ws/build

# Utility rule file for gui_other_files.

# Include the progress variables for this target.
include gui/CMakeFiles/gui_other_files.dir/progress.make

gui_other_files: gui/CMakeFiles/gui_other_files.dir/build.make

.PHONY : gui_other_files

# Rule to build all files generated by this target.
gui/CMakeFiles/gui_other_files.dir/build: gui_other_files

.PHONY : gui/CMakeFiles/gui_other_files.dir/build

gui/CMakeFiles/gui_other_files.dir/clean:
	cd /home/ameyasu/QTprojects/scanner-gui/gui_ws/build/gui && $(CMAKE_COMMAND) -P CMakeFiles/gui_other_files.dir/cmake_clean.cmake
.PHONY : gui/CMakeFiles/gui_other_files.dir/clean

gui/CMakeFiles/gui_other_files.dir/depend:
	cd /home/ameyasu/QTprojects/scanner-gui/gui_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ameyasu/QTprojects/scanner-gui/gui_ws/src /home/ameyasu/QTprojects/scanner-gui/gui_ws/src/gui /home/ameyasu/QTprojects/scanner-gui/gui_ws/build /home/ameyasu/QTprojects/scanner-gui/gui_ws/build/gui /home/ameyasu/QTprojects/scanner-gui/gui_ws/build/gui/CMakeFiles/gui_other_files.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gui/CMakeFiles/gui_other_files.dir/depend
