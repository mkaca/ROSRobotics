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
CMAKE_SOURCE_DIR = /home/mike/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mike/catkin_ws/src

# Utility rule file for astra_openni2.

# Include the progress variables for this target.
include astra_camera/CMakeFiles/astra_openni2.dir/progress.make

astra_camera/CMakeFiles/astra_openni2: astra_camera/CMakeFiles/astra_openni2-complete


astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-install
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-mkdir
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-update
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-patch
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-build
astra_camera/CMakeFiles/astra_openni2-complete: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/CMakeFiles
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/CMakeFiles/astra_openni2-complete
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-done

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-install: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && tar -xjf /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2/Packaging/Final/OpenNI-Linux-2.3.tar.bz2 -C /home/mike/catkin_ws/src/astra_camera/openni2 --strip 1 && mkdir -p /home/mike/catkin_ws/src/astra_camera/openni2/include && ln -fs /home/mike/catkin_ws/src/astra_camera/openni2/Include /home/mike/catkin_ws/src/astra_camera/openni2/include/openni2
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-install

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/openni2
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/astra_openni2/tmp
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E make_directory /home/mike/catkin_ws/src/astra_camera/astra_openni2/src
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-mkdir

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-gitinfo.txt
astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src && /usr/bin/cmake -P /home/mike/catkin_ws/src/astra_camera/astra_openni2/tmp/astra_openni2-gitclone.cmake
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-update: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && /usr/bin/cmake -P /home/mike/catkin_ws/src/astra_camera/astra_openni2/tmp/astra_openni2-gitupdate.cmake

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-patch: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E echo_append
	cd /home/mike/catkin_ws/src/astra_camera && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-patch

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure: astra_camera/astra_openni2/tmp/astra_openni2-cfgcmd.txt
astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-update
astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && echo "no need to configure"
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure

astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-build: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mike/catkin_ws/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'astra_openni2'"
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && make release FILTER=On ALLOW_WARNINGS=1
	cd /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2 && /usr/bin/cmake -E touch /home/mike/catkin_ws/src/astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-build

astra_openni2: astra_camera/CMakeFiles/astra_openni2
astra_openni2: astra_camera/CMakeFiles/astra_openni2-complete
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-install
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-mkdir
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-download
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-update
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-patch
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-configure
astra_openni2: astra_camera/astra_openni2/src/astra_openni2-stamp/astra_openni2-build
astra_openni2: astra_camera/CMakeFiles/astra_openni2.dir/build.make

.PHONY : astra_openni2

# Rule to build all files generated by this target.
astra_camera/CMakeFiles/astra_openni2.dir/build: astra_openni2

.PHONY : astra_camera/CMakeFiles/astra_openni2.dir/build

astra_camera/CMakeFiles/astra_openni2.dir/clean:
	cd /home/mike/catkin_ws/src/astra_camera && $(CMAKE_COMMAND) -P CMakeFiles/astra_openni2.dir/cmake_clean.cmake
.PHONY : astra_camera/CMakeFiles/astra_openni2.dir/clean

astra_camera/CMakeFiles/astra_openni2.dir/depend:
	cd /home/mike/catkin_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mike/catkin_ws/src /home/mike/catkin_ws/src/astra_camera /home/mike/catkin_ws/src /home/mike/catkin_ws/src/astra_camera /home/mike/catkin_ws/src/astra_camera/CMakeFiles/astra_openni2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : astra_camera/CMakeFiles/astra_openni2.dir/depend

