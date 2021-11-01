TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build a Diamond Support Module
# -------------------------------

LIBRARY_IOC += ADElcomat

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
DBD += ADElcomat.dbd

# The following are compiled and added to the support library
ADElcomat_SRCS += ADElcomat.cpp

# We need to link against the EPICS Base libraries
#adfemApp_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

include $(TOP)/configure/RULES

