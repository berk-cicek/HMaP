BASE = ../botop/rai
BASE2 = ../botop/src

# List all the source files you need to compile
SRCS = main.cpp \
       $(BASE)/src/Control/TimingMPC.cpp \
       $(BASE)/src/Control/WaypointMPC.cpp \
       $(BASE)/src/Control/ShortPathMPC.cpp \
       $(BASE)/src/KOMO/manipTools.cpp \
       $(BASE)/src/Algo/SplineCtrlFeed.cpp \
       $(BASE)/src/Control/timingOpt.cpp

OBJS = $(SRCS:.cpp=.o)

# Add BASE, BASE2, and BASE/src/Control to the include paths
CXXFLAGS += -I$(BASE) -I$(BASE2) -I$(BASE)/src/Control

DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control

include $(BASE)/_make/generic.mk

# Define rules for compiling object files from source files
.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Define the final target
all: $(OBJS)
	$(CXX) $(CXXFLAGS) -o my_program $(OBJS) $(LDFLAGS)

