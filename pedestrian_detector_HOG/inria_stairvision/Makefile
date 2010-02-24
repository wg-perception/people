# Makefile for stairvision / inria conversion tools
# Ian Goodfellow <ia3n@cs.stanford.edu>

LASIK_PATH := $(shell rospack find stairvision_latest)/build/stairvision-svn

USE_OPENCV = 1
USE_GSL = 0
USE_LAPACK = 0

-include $(LASIK_PATH)/make.mk
#-include $(LASIK_PATH)/projects/person/make.person_common

MY_BIN_PATH=$(shell pwd)/bin

#######################################################################
# add application source files here
#######################################################################
SVL_CLONED_APPS = buildPatchResponseCache.cpp classifyImages.cpp \
 extractorCat.cpp 
APP_SRC = ${SVL_CLONED_APPS} cvtest.cpp defaultHOG.cpp svl2inria.cpp

OTHER_SRC = hogWrapper.cpp

#######################################################################

APP_PROG_NAMES = $(APP_SRC:.cpp=)
APP_OBJ = $(APP_SRC:.cpp=.o)
OTHER_OBJ = $(OTHER_SRC:.cpp=.o)

LIB_CFLAGS = -I$(LASIK_PATH)/lib -I$(LASIK_PATH)/lib/core -I$(LASIK_PATH)/lib/ml -I$(LASIK_PATH)/lib/utils
EXT_CFLAGS = -I$(EXT_PATH)/xmlParser -I$(EXT_PATH)
EXT_LFLAGS = $(EXT_PATH)/xmlParser/xmlParser.a  $(EXT_PATH)/lbfgs/lbfgs.a

CFLAGS += $(WX_CFLAGS) $(EXT_CFLAGS) $(LIB_CFLAGS)
LFLAGS += $(WX_LFLAGS) $(EXT_LFLAGS) $(LIBSVL)  `rospack find opencv_svn`/build/opencv-svn/opencv/build/lib/libcvaux-new.so.1.1.0
#`rospack find opencv2`/build/opencv-svn/build/lib/libcvaux.so.1.1.0

.PHONY: clean
.PRECIOUS: $(APP_OBJ)

all: depend ${addprefix ${MY_BIN_PATH}/,$(APP_PROG_NAMES)}

$(MY_BIN_PATH)/%: %.o $(OTHER_OBJ) $(LIBSVL)
	${CCC} $*.o -o $(@:.o=) $(OTHER_OBJ) $(LIBSVL) $(LFLAGS)

$(LIBSVL):
	@echo "YOU NEED TO MAKE THE LIBLASIK LIBRARY FIRST"
	false

.cpp.o:
	${CCC} ${CFLAGS} -c $< -o $@

depend:
	echo ${LASIK_PATH}
	g++ ${CFLAGS} -MM ${APP_SRC} ${OTHER_SRC} >depend

clean:
	@echo "Removing app objects"
	-rm $(APP_OBJ)
	@echo "Removing non-executable objects"
	-rm $(OTHER_OBJ)
	@echo "Removing apps"
	-rm ${addprefix ${MY_BIN_PATH}/,$(APP_PROG_NAMES)} 
	-rm depend 

-include depend

