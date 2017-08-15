$(shell mkdir -p .build/ 2>/dev/null)
$(shell mkdir -p bin/ 2>/dev/null)

SRC = $(wildcard src/*.cpp)
TST = $(wildcard test/*.cpp)
SOBJ = $(patsubst %.cpp, .build/%.o, $(notdir $(SRC)))
TOBJ = $(patsubst %.cpp, .build/%.o, $(notdir $(TST)))
DEP = $(patsubst %.o, %.d, $(SOBJ) $(TOBJ))

CC = g++
CF = -g -std=c++11 -Wall -Wpedantic
LF = -g
LIB = -lncurses -lwiringPi
INC = -I include/

all: launch.exe

launch.exe: $(SOBJ)
	$(CC) $(LF) $(SOBJ) -o $@ $(LIB)

%.o: %.d
	$(CC) $(CF) -c $(patsubst .build/%.o, src/%.cpp, $@) $(INC) -o $@
	@touch $@

.build/%.d: src/%.cpp
	$(CC) -std=c++11 -MM $< -MT $@ -MP -MF $@ $(INC)

.build/%.d: test/%.cpp
	$(CC) -std=c++11 -MM $< -MT $@ -MP -MF $@ $(INC)

clean:
	-@rm -r -f .build 2>/dev/null || true
	-@rm -r -f bin 2>/dev/null || true
	-@rm launch.exe 2>/dev/null || true

-include $(DEP)
