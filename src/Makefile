COMPILER = g++
CFLAGS = -Isrc
LDFLAGS = -lraylib -lreactphysics3d -static-libstdc++ -static-libgcc

TARGET = ../robot_emulator
OBJS = main.o robot_emulator.o robot.o example_robot.o

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
robot_emulator.o: robot_emulator.cpp robot_emulator.h robot.cpp robot.h
robot.o: robot.cpp robot.h robot_emulator.cpp robot_emulator.h
example_robot.o: example_robot.cpp example_robot.h robot.cpp robot.h robot_emulator.cpp robot_emulator.h

clean:
	rm *.o
	rm $(TARGET)
