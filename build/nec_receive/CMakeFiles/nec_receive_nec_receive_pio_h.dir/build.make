# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\cmake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\cmake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\development\remote-control-car

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\development\remote-control-car\build

# Utility rule file for nec_receive_nec_receive_pio_h.

# Include any custom commands dependencies for this target.
include nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/compiler_depend.make

# Include the progress variables for this target.
include nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/progress.make

nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h: nec_receive/nec_receive.pio.h

nec_receive/nec_receive.pio.h: C:/development/remote-control-car/nec_receive/nec_receive.pio
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\development\remote-control-car\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nec_receive.pio.h"
	cd /d C:\development\remote-control-car\build\nec_receive && "C:\Program Files\Raspberry Pi\Pico SDK v1.5.1\pico-sdk-tools\pioasm.exe" -o c-sdk C:/development/remote-control-car/nec_receive/nec_receive.pio C:/development/remote-control-car/build/nec_receive/nec_receive.pio.h

nec_receive_nec_receive_pio_h: nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h
nec_receive_nec_receive_pio_h: nec_receive/nec_receive.pio.h
nec_receive_nec_receive_pio_h: nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/build.make
.PHONY : nec_receive_nec_receive_pio_h

# Rule to build all files generated by this target.
nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/build: nec_receive_nec_receive_pio_h
.PHONY : nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/build

nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/clean:
	cd /d C:\development\remote-control-car\build\nec_receive && $(CMAKE_COMMAND) -P CMakeFiles\nec_receive_nec_receive_pio_h.dir\cmake_clean.cmake
.PHONY : nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/clean

nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\development\remote-control-car C:\development\remote-control-car\nec_receive C:\development\remote-control-car\build C:\development\remote-control-car\build\nec_receive C:\development\remote-control-car\build\nec_receive\CMakeFiles\nec_receive_nec_receive_pio_h.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : nec_receive/CMakeFiles/nec_receive_nec_receive_pio_h.dir/depend

