# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/pi/rpi-uav

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/rpi-uav

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/rpi-uav/CMakeFiles /home/pi/rpi-uav/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/rpi-uav/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named skips

# Build rule for target.
skips: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 skips
.PHONY : skips

# fast build rule for target.
skips/fast:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/build
.PHONY : skips/fast

#=============================================================================
# Target rules for targets named bin2vel

# Build rule for target.
bin2vel: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 bin2vel
.PHONY : bin2vel

# fast build rule for target.
bin2vel/fast:
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/build
.PHONY : bin2vel/fast

#=============================================================================
# Target rules for targets named bin2txt

# Build rule for target.
bin2txt: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 bin2txt
.PHONY : bin2txt

# fast build rule for target.
bin2txt/fast:
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/build
.PHONY : bin2txt/fast

src/logging/uavcore.o: src/logging/uavcore.cpp.o

.PHONY : src/logging/uavcore.o

# target to build an object file
src/logging/uavcore.cpp.o:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/src/logging/uavcore.cpp.o
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/src/logging/uavcore.cpp.o
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/src/logging/uavcore.cpp.o
.PHONY : src/logging/uavcore.cpp.o

src/logging/uavcore.i: src/logging/uavcore.cpp.i

.PHONY : src/logging/uavcore.i

# target to preprocess a source file
src/logging/uavcore.cpp.i:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/src/logging/uavcore.cpp.i
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/src/logging/uavcore.cpp.i
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/src/logging/uavcore.cpp.i
.PHONY : src/logging/uavcore.cpp.i

src/logging/uavcore.s: src/logging/uavcore.cpp.s

.PHONY : src/logging/uavcore.s

# target to generate assembly for a file
src/logging/uavcore.cpp.s:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/src/logging/uavcore.cpp.s
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/src/logging/uavcore.cpp.s
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/src/logging/uavcore.cpp.s
.PHONY : src/logging/uavcore.cpp.s

src/math/freebody.o: src/math/freebody.cpp.o

.PHONY : src/math/freebody.o

# target to build an object file
src/math/freebody.cpp.o:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/src/math/freebody.cpp.o
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/src/math/freebody.cpp.o
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/src/math/freebody.cpp.o
.PHONY : src/math/freebody.cpp.o

src/math/freebody.i: src/math/freebody.cpp.i

.PHONY : src/math/freebody.i

# target to preprocess a source file
src/math/freebody.cpp.i:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/src/math/freebody.cpp.i
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/src/math/freebody.cpp.i
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/src/math/freebody.cpp.i
.PHONY : src/math/freebody.cpp.i

src/math/freebody.s: src/math/freebody.cpp.s

.PHONY : src/math/freebody.s

# target to generate assembly for a file
src/math/freebody.cpp.s:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/src/math/freebody.cpp.s
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/src/math/freebody.cpp.s
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/src/math/freebody.cpp.s
.PHONY : src/math/freebody.cpp.s

test/bin2txt.o: test/bin2txt.cpp.o

.PHONY : test/bin2txt.o

# target to build an object file
test/bin2txt.cpp.o:
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/test/bin2txt.cpp.o
.PHONY : test/bin2txt.cpp.o

test/bin2txt.i: test/bin2txt.cpp.i

.PHONY : test/bin2txt.i

# target to preprocess a source file
test/bin2txt.cpp.i:
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/test/bin2txt.cpp.i
.PHONY : test/bin2txt.cpp.i

test/bin2txt.s: test/bin2txt.cpp.s

.PHONY : test/bin2txt.s

# target to generate assembly for a file
test/bin2txt.cpp.s:
	$(MAKE) -f CMakeFiles/bin2txt.dir/build.make CMakeFiles/bin2txt.dir/test/bin2txt.cpp.s
.PHONY : test/bin2txt.cpp.s

test/bin2vel.o: test/bin2vel.cpp.o

.PHONY : test/bin2vel.o

# target to build an object file
test/bin2vel.cpp.o:
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/test/bin2vel.cpp.o
.PHONY : test/bin2vel.cpp.o

test/bin2vel.i: test/bin2vel.cpp.i

.PHONY : test/bin2vel.i

# target to preprocess a source file
test/bin2vel.cpp.i:
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/test/bin2vel.cpp.i
.PHONY : test/bin2vel.cpp.i

test/bin2vel.s: test/bin2vel.cpp.s

.PHONY : test/bin2vel.s

# target to generate assembly for a file
test/bin2vel.cpp.s:
	$(MAKE) -f CMakeFiles/bin2vel.dir/build.make CMakeFiles/bin2vel.dir/test/bin2vel.cpp.s
.PHONY : test/bin2vel.cpp.s

test/skips.o: test/skips.cpp.o

.PHONY : test/skips.o

# target to build an object file
test/skips.cpp.o:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/test/skips.cpp.o
.PHONY : test/skips.cpp.o

test/skips.i: test/skips.cpp.i

.PHONY : test/skips.i

# target to preprocess a source file
test/skips.cpp.i:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/test/skips.cpp.i
.PHONY : test/skips.cpp.i

test/skips.s: test/skips.cpp.s

.PHONY : test/skips.s

# target to generate assembly for a file
test/skips.cpp.s:
	$(MAKE) -f CMakeFiles/skips.dir/build.make CMakeFiles/skips.dir/test/skips.cpp.s
.PHONY : test/skips.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... skips"
	@echo "... bin2vel"
	@echo "... bin2txt"
	@echo "... src/logging/uavcore.o"
	@echo "... src/logging/uavcore.i"
	@echo "... src/logging/uavcore.s"
	@echo "... src/math/freebody.o"
	@echo "... src/math/freebody.i"
	@echo "... src/math/freebody.s"
	@echo "... test/bin2txt.o"
	@echo "... test/bin2txt.i"
	@echo "... test/bin2txt.s"
	@echo "... test/bin2vel.o"
	@echo "... test/bin2vel.i"
	@echo "... test/bin2vel.s"
	@echo "... test/skips.o"
	@echo "... test/skips.i"
	@echo "... test/skips.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

