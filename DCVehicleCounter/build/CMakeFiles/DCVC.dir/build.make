# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build

# Include any dependencies generated for this target.
include CMakeFiles/DCVC.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DCVC.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DCVC.dir/flags.make

CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o: ../BackgroundProcessor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/BackgroundProcessor.cpp

CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/BackgroundProcessor.cpp > CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.i

CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/BackgroundProcessor.cpp -o CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.s

CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.requires

CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.provides: CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.provides

CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.provides.build: CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o

CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o: ../ImageEnhancer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ImageEnhancer.cpp

CMakeFiles/DCVC.dir/ImageEnhancer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/ImageEnhancer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ImageEnhancer.cpp > CMakeFiles/DCVC.dir/ImageEnhancer.cpp.i

CMakeFiles/DCVC.dir/ImageEnhancer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/ImageEnhancer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ImageEnhancer.cpp -o CMakeFiles/DCVC.dir/ImageEnhancer.cpp.s

CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.requires

CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.provides: CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.provides

CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.provides.build: CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o

CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o: ../ObjectSplitterVehicleCounter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ObjectSplitterVehicleCounter.cpp

CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ObjectSplitterVehicleCounter.cpp > CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.i

CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ObjectSplitterVehicleCounter.cpp -o CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.s

CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.requires

CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.provides: CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.provides

CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.provides.build: CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o

CMakeFiles/DCVC.dir/Resizer.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/Resizer.cpp.o: ../Resizer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/Resizer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/Resizer.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/Resizer.cpp

CMakeFiles/DCVC.dir/Resizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/Resizer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/Resizer.cpp > CMakeFiles/DCVC.dir/Resizer.cpp.i

CMakeFiles/DCVC.dir/Resizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/Resizer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/Resizer.cpp -o CMakeFiles/DCVC.dir/Resizer.cpp.s

CMakeFiles/DCVC.dir/Resizer.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/Resizer.cpp.o.requires

CMakeFiles/DCVC.dir/Resizer.cpp.o.provides: CMakeFiles/DCVC.dir/Resizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/Resizer.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/Resizer.cpp.o.provides

CMakeFiles/DCVC.dir/Resizer.cpp.o.provides.build: CMakeFiles/DCVC.dir/Resizer.cpp.o

CMakeFiles/DCVC.dir/VehicleTracking.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/VehicleTracking.cpp.o: ../VehicleTracking.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/VehicleTracking.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/VehicleTracking.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/VehicleTracking.cpp

CMakeFiles/DCVC.dir/VehicleTracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/VehicleTracking.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/VehicleTracking.cpp > CMakeFiles/DCVC.dir/VehicleTracking.cpp.i

CMakeFiles/DCVC.dir/VehicleTracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/VehicleTracking.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/VehicleTracking.cpp -o CMakeFiles/DCVC.dir/VehicleTracking.cpp.s

CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.requires

CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.provides: CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.provides

CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.provides.build: CMakeFiles/DCVC.dir/VehicleTracking.cpp.o

CMakeFiles/DCVC.dir/VehicleMerger.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/VehicleMerger.cpp.o: ../VehicleMerger.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/VehicleMerger.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/VehicleMerger.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/VehicleMerger.cpp

CMakeFiles/DCVC.dir/VehicleMerger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/VehicleMerger.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/VehicleMerger.cpp > CMakeFiles/DCVC.dir/VehicleMerger.cpp.i

CMakeFiles/DCVC.dir/VehicleMerger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/VehicleMerger.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/VehicleMerger.cpp -o CMakeFiles/DCVC.dir/VehicleMerger.cpp.s

CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.requires

CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.provides: CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.provides

CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.provides.build: CMakeFiles/DCVC.dir/VehicleMerger.cpp.o

CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o: ../ObjectSplitter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ObjectSplitter.cpp

CMakeFiles/DCVC.dir/ObjectSplitter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/ObjectSplitter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ObjectSplitter.cpp > CMakeFiles/DCVC.dir/ObjectSplitter.cpp.i

CMakeFiles/DCVC.dir/ObjectSplitter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/ObjectSplitter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/ObjectSplitter.cpp -o CMakeFiles/DCVC.dir/ObjectSplitter.cpp.s

CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.requires

CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.provides: CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.provides

CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.provides.build: CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o

CMakeFiles/DCVC.dir/EdgeDetector.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/EdgeDetector.cpp.o: ../EdgeDetector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/EdgeDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/EdgeDetector.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/EdgeDetector.cpp

CMakeFiles/DCVC.dir/EdgeDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/EdgeDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/EdgeDetector.cpp > CMakeFiles/DCVC.dir/EdgeDetector.cpp.i

CMakeFiles/DCVC.dir/EdgeDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/EdgeDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/EdgeDetector.cpp -o CMakeFiles/DCVC.dir/EdgeDetector.cpp.s

CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.requires

CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.provides: CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.provides

CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.provides.build: CMakeFiles/DCVC.dir/EdgeDetector.cpp.o

CMakeFiles/DCVC.dir/Blinker.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/Blinker.cpp.o: ../Blinker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/Blinker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/Blinker.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/Blinker.cpp

CMakeFiles/DCVC.dir/Blinker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/Blinker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/Blinker.cpp > CMakeFiles/DCVC.dir/Blinker.cpp.i

CMakeFiles/DCVC.dir/Blinker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/Blinker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/Blinker.cpp -o CMakeFiles/DCVC.dir/Blinker.cpp.s

CMakeFiles/DCVC.dir/Blinker.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/Blinker.cpp.o.requires

CMakeFiles/DCVC.dir/Blinker.cpp.o.provides: CMakeFiles/DCVC.dir/Blinker.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/Blinker.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/Blinker.cpp.o.provides

CMakeFiles/DCVC.dir/Blinker.cpp.o.provides.build: CMakeFiles/DCVC.dir/Blinker.cpp.o

CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o: ../SaveLoadConfig.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/SaveLoadConfig.cpp

CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/SaveLoadConfig.cpp > CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.i

CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/SaveLoadConfig.cpp -o CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.s

CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.requires

CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.provides: CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.provides

CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.provides.build: CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o

CMakeFiles/DCVC.dir/main.cpp.o: CMakeFiles/DCVC.dir/flags.make
CMakeFiles/DCVC.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DCVC.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/DCVC.dir/main.cpp.o -c /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/main.cpp

CMakeFiles/DCVC.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DCVC.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/main.cpp > CMakeFiles/DCVC.dir/main.cpp.i

CMakeFiles/DCVC.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DCVC.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/main.cpp -o CMakeFiles/DCVC.dir/main.cpp.s

CMakeFiles/DCVC.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/DCVC.dir/main.cpp.o.requires

CMakeFiles/DCVC.dir/main.cpp.o.provides: CMakeFiles/DCVC.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/DCVC.dir/build.make CMakeFiles/DCVC.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/DCVC.dir/main.cpp.o.provides

CMakeFiles/DCVC.dir/main.cpp.o.provides.build: CMakeFiles/DCVC.dir/main.cpp.o

# Object files for target DCVC
DCVC_OBJECTS = \
"CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o" \
"CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o" \
"CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o" \
"CMakeFiles/DCVC.dir/Resizer.cpp.o" \
"CMakeFiles/DCVC.dir/VehicleTracking.cpp.o" \
"CMakeFiles/DCVC.dir/VehicleMerger.cpp.o" \
"CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o" \
"CMakeFiles/DCVC.dir/EdgeDetector.cpp.o" \
"CMakeFiles/DCVC.dir/Blinker.cpp.o" \
"CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o" \
"CMakeFiles/DCVC.dir/main.cpp.o"

# External object files for target DCVC
DCVC_EXTERNAL_OBJECTS =

DCVC: CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o
DCVC: CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o
DCVC: CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o
DCVC: CMakeFiles/DCVC.dir/Resizer.cpp.o
DCVC: CMakeFiles/DCVC.dir/VehicleTracking.cpp.o
DCVC: CMakeFiles/DCVC.dir/VehicleMerger.cpp.o
DCVC: CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o
DCVC: CMakeFiles/DCVC.dir/EdgeDetector.cpp.o
DCVC: CMakeFiles/DCVC.dir/Blinker.cpp.o
DCVC: CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o
DCVC: CMakeFiles/DCVC.dir/main.cpp.o
DCVC: CMakeFiles/DCVC.dir/build.make
DCVC: /usr/local/lib/libopencv_core.so.2.4.9
DCVC: /usr/local/lib/libopencv_highgui.so.2.4.9
DCVC: /usr/local/lib/libopencv_imgproc.so.2.4.9
DCVC: /usr/local/lib/libopencv_video.so.2.4.9
DCVC: /usr/local/lib/libopencv_imgproc.so.2.4.9
DCVC: /usr/local/lib/libopencv_core.so.2.4.9
DCVC: CMakeFiles/DCVC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable DCVC"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DCVC.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DCVC.dir/build: DCVC
.PHONY : CMakeFiles/DCVC.dir/build

CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/BackgroundProcessor.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/ImageEnhancer.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/ObjectSplitterVehicleCounter.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/Resizer.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/VehicleTracking.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/VehicleMerger.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/ObjectSplitter.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/EdgeDetector.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/Blinker.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/SaveLoadConfig.cpp.o.requires
CMakeFiles/DCVC.dir/requires: CMakeFiles/DCVC.dir/main.cpp.o.requires
.PHONY : CMakeFiles/DCVC.dir/requires

CMakeFiles/DCVC.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DCVC.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DCVC.dir/clean

CMakeFiles/DCVC.dir/depend:
	cd /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build /home/yakhe/work/DCVehicleCounter/DCVehicleCounter/DCVehicleCounter/build/CMakeFiles/DCVC.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DCVC.dir/depend
