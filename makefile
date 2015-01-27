
#
# make - Names
#

    MAKE_NAME=poco2pano

#
# make - Directories
#

    MAKE_BINARY=bin
    MAKE_SOURCE=src
    MAKE_OBJECT=obj

#
# make - Files
#

    MAKE_SCC=$(wildcard $(MAKE_SOURCE)/*.c)
    MAKE_SCP=$(wildcard $(MAKE_SOURCE)/*.cpp)
    MAKE_OBC=$(addprefix $(MAKE_OBJECT)/, $(addsuffix .o, $(notdir $(basename $(MAKE_SCC)))))
    MAKE_OBP=$(addprefix $(MAKE_OBJECT)/, $(addsuffix .o, $(notdir $(basename $(MAKE_SCP)))))

#
# make - Constants
#

    MAKE_CCC=gcc
    MAKE_CCP=g++
    MAKE_LKD=gcc
    MAKE_OPC=-Wall -c -funsigned-char -std=c99
    MAKE_OPP=-Wall -c -funsigned-char -O3 -static -s
    MAKE_OPI=
    MAKE_OPL=-lstdc++ -lm
#
# make - All
#

    all:directories $(MAKE_NAME)

#
# make - Binaries
#

    $(MAKE_NAME): $(MAKE_OBP)
	$(MAKE_LKD) -o $(MAKE_BINARY)/$(MAKE_NAME) $^ $(MAKE_OPL)

#
# make - Objects
#

#    $(MAKE_OBJECT)/%.o:$(MAKE_SOURCE)/%.c
#$(MAKE_CCC) $(MAKE_OPC) -o $@ $<

    $(MAKE_OBJECT)/%.o:$(MAKE_SOURCE)/%.cpp
	$(MAKE_CCP) $(MAKE_OPI) $(MAKE_OPP) -o $@ $<

#
# make - Directories
#

    directories:
	mkdir -p $(MAKE_OBJECT)
	mkdir -p $(MAKE_BINARY)

#
# make - Cleaning
#

    clean:
	rm $(MAKE_BINARY)/* -f
	rm $(MAKE_OBJECT)/*.o -f
