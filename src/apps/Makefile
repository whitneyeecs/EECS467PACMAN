include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 $(CFLAGS_A3) \
	 -O2

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES)\
	  $(LDFLAGS_A3)

CXXFLAGS =$(CXXFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_A3) \
	 $(CFLAGS_LCMTYPES) \
	 -O2


LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o

BIN_EECS467_GUI_EXAMPLE = $(BIN_PATH)/eecs467_gui_example
BIN_EECS467_REXARM_EXAMPLE = $(BIN_PATH)/eecs467_rexarm_example
BIN_EECS467_MAEBOT_EXAMPLE = $(BIN_PATH)/eecs467_maebot_example
BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop
BIN_PACMAN = $(BIN_PATH)/pacman
BIN_GAME = $(BIN_PATH)/game
BIN_BOARDTEST = $(BIN_PATH)/boardTest

ALL = $(LIB_EECS467) $(BIN_EECS467_GUI_EXAMPLE) $(BIN_EECS467_REXARM_EXAMPLE) \
      $(BIN_EECS467_MAEBOT_EXAMPLE) $(BIN_EECS467_MAEBOT_VIDEO_TELOP) \
	  $(BIN_PACMAN)	$(BIN_GAME) $(BIN_BOARDTEST)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_EECS467_GUI_EXAMPLE): gui_example.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_EECS467_REXARM_EXAMPLE): rexarm_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_EXAMPLE): maebot_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_VIDEO_TELOP): maebot_video_teleop.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PACMAN): pacman.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_GAME): game.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_BOARDTEST): boardTest.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
