COMPILER = g++
CFLAGS = -Isrc
LDFLAGS = -lraylib -lreactphysics3d -static-libstdc++ -static-libgcc

TARGET = ../robot_emulator
OBJS = main.o robot_emulator.o util.o robot.o example_robot.o
DEH = robot_emulator.cpp robot_emulator.h util.cpp util.h

%.o: %.cpp
	@echo compiling $<
	@$(COMPILER) $(CFLAGS) -c -o $@ $<
$(TARGET): $(OBJS)
	@echo making exe
	@$(COMPILER) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)
debug:
	$(COMPILER) $(CFLAGS) -c *.cpp -g
	$(COMPILER) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS) -g

main.o: *.cpp *.h
robot_emulator.o: robot_emulator.cpp robot_emulator.h robot.cpp robot.h util.cpp util.h
util.o: util.cpp util.h robot_emulator.cpp robot_emulator.h
robot.o: robot.cpp robot.h $(DEH)
example_robot.o: example_robot.cpp example_robot.h robot.cpp robot.h $(DEH)

clean:
	rm *.o
	rm $(TARGET)
