# To make ROBOT=HRP3STEP2
# The makefile in which the target plugin is actually made is found in /Sample/IOServer/Make.rules.common
include Makefile.robot
all: target

# Parent directory setings
#include ../Makefile.setup

#--------------------------------------------- Locations  -----------------------------------------------------------
TOP = /home/grxuser/src/OpenHRP3.0/
DYNAMICSSIMULATOR = $(TOP)DynamicsSimulator/

# Rules                                                                                                                                 
include $(TOP)Controller/IOserver/plugin/Make.rules
#--------------------------------------------- Compile Commands -----------------------------------------------------------
CXX_FLAGS += -g -Wall -O0										# Compilation Flags
CXX_FLAGS:=$(CXX_FLAGS) -I/usr/include/qdbm
CXX_FLAGS += -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing
#CXX_FLAGS += -DDEBUG_PLUGIN -DDEBUG_PLUGIN2 -DDEBUG_PLUGIN3			 	#// used in forceSensorPlugin=#1 // hiroArm=#2 // AssemblyStrat=3 respectively
CXX_FLAGS += -DPIVOTAPPROACH #-DIMPEDANCE 
CXX_FLAGS += -DSIMULATION
CXX_FLAGS += -DWRITELOG
CXX_FLAGS += -I$(TOP)Controller/IOserver/include   
CXX_FLAGS += -I$(TOP)Controller/IOserver/robot/HRP2STEP1/
CXX_FLAGS += -I$(TOP)Common -I$(TOP)Controller/IOserver/robot/HRP2STEP1/iob/ 
CXX_FLAGS += -I$(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/
CXX_FLAGS += -I../corba -I$(TOP)DynamicsSimulator/server -I$(MODELLOADER)/$(CORBA_DIR)

#------------------------------------------ Linking Commands  --------------------------------------------------------
LINK += -L$(TOP)Controller/IOserver/robot/HRP2STEP1/bin 
LINK += -L$(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/
LINK += #-lAssemblyStrategy -lFilterTools

#CXX_FLAGS += -I/usr/include/boost
#CXX_FLAGS += -I$(HOME)/src/Matrix/boost-cvs/boost-sandbox
#CXX_FLAGS += -I$(TOP)Controller/IOserver/plugin/IoControlPlugin/$(CORBA_DIR)


#------------------------------------------ QNX --------------------------------------------------------
## If the operating system is QNX, fix the flags
#ifneq ($(OSNAME),QNX)
	CXX_FLAGS += -pg -ggdb3
#else
#	CXX_FLAGS += -ggdb3
#endif

#------------------------------------------ Objects --------------------------------------------------------
forceSensorPlugin_impl.cpp: forceSensorPlugin_impl.h
forceSensorPlugin_impl.o: forceSensorPlugin_impl.cpp

$(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/ifs_com.c: $(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/ifs_com.h
ifs_com.o: $(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/ifs_com.c
	$(CC) $(CXX_FLAGS) -c $<

$(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/ifs.c: $(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/ifs.h
ifs.o: $(TOP)Controller/IOserver/robot/HRP2STEP1/iob/hrp_servo/ifspci/ifs.c
	$(CC) $(CXX_FLAGS) -c $<

hiroArm.cpp: hiroArm.h
hiroArm.o: hiroArm.cpp

FilterTools.cpp: FilterTools.h
FilterTools.o: FilterTools.cpp

ControlBasis.cpp: ControlBasis.h
ControlBasis.o: ControlBasis.cpp

AssemblyStrategy.cpp: AssemblyStrategy.h
AssemblyStrategy.o: AssemblyStrategy.cpp

#---------------------------------- forceSensor_Plugin Executable -------------------------------------------
$(PLUGIN): forceSensorPlugin_impl.o ifs.o ifs_com.o hiroArm.o FilterTools.o ControlBasis.o AssemblyStrategy.o # Object and Library files

#-------------------------------------------- Clean  -------------------------------------------
rmversionString:
	-$(RM) forceSensorPlugin_impl.o  hiroArm.o ifs.o ifs_com.o FilterTools.o ControlBasis.o AssemblyStrategy.o
	-$(RM) $(PLUGIN)
#-----------------------------------------------------------------------------------------------------------
