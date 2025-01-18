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
       $(BASE3)/HMAPBiman.cpp   # Add your new class source file here

OBJS = $(SRCS:.cpp=.o)

# Add BASE, BASE2, BASE/src/Control, and OpenCV include paths
CXXFLAGS += -I$(BASE) -I$(BASE2) -I$(BASE3) -I$(BASE)/src/Control -I/usr/include/opencv4

# Add the OpenCV libraries and pthread to the linking step
LDFLAGS += `pkg-config --libs opencv4` -pthread

# Dependencies for linking
DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control

# Include generic make rules
include $(BASE)/_make/generic.mk

# Define rules for compiling object files from source files
.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Define the final target
all: $(OBJS)
	$(CXX) $(CXXFLAGS) -o my_program $(OBJS) $(LDFLAGS)
