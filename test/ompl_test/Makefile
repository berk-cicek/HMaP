# Define the base directories for your project
BASE = ../../../botop/rai
BASE2 = ../../../botop/src
BASE3 = ../../src

# List all the source files you need to compile
SRCS = main.cpp \
       $(BASE)/src/Control/TimingMPC.cpp \
       $(BASE)/src/Control/WaypointMPC.cpp \
       $(BASE)/src/Control/ShortPathMPC.cpp \
       $(BASE)/src/KOMO/manipTools.cpp \
       $(BASE)/src/Algo/SplineCtrlFeed.cpp \
       $(BASE)/src/Control/timingOpt.cpp \
       $(BASE3)/HMAPBiman.cpp \
       $(BASE3)/ompl_test.cpp

# Object files from source files
OBJS = $(SRCS:.cpp=.o)

# Add include directories for BASE, BASE2, BASE/src/Control, OpenCV, and OMPL
CXXFLAGS += -I$(BASE) -I$(BASE2) -I$(BASE3) -I$(BASE)/src/Control \
            -I/usr/include/opencv4 -I/usr/local/include/ompl-1.6 -I/usr/include/eigen3

LPATHS  += /usr/local/lib  # Add OMPL library path
LIBS    += -lrt -lompl     # Ensure OMPL is included in libraries

LDFLAGS += $(LPATHS:%=-L%) # Add library paths
LDFLAGS += -lompl          # Ensure OMPL is included in linking


# Add OpenCV and OMPL libraries to the linking step
LDFLAGS += `pkg-config --libs opencv4`

# Define dependencies for linking
DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control OMPL  # OMPL added to dependencies

# Include the generic make rules
include $(BASE)/_make/generic.mk

# Rule for compiling object files from source files
.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Final target definition
all: $(OBJS)
	$(CXX) $(CXXFLAGS) -o my_program $(OBJS) $(LDFLAGS)

# Clean target to remove object files and binaries
clean:
	rm -f $(OBJS) my_program
