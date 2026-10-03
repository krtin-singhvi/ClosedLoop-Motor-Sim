CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 \
           -Icomponents \
           -Ioutput \
           -Isimulation \
           -MMD -MP

TARGET = main

SOURCES = main.cpp \
          components/motor.cpp \
          components/pid_controller.cpp \
          output/consoleoutput.cpp \
          output/csvoutput.cpp \
          simulation/device.cpp \
          simulation/simulationresult.cpp \
          simulation/simulator.cpp \
		  simulation/variation.cpp \

OBJECTS = $(SOURCES:.cpp=.o)

DEPENDS = $(OBJECTS:.o=.d)


$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET)


%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


-include $(DEPENDS)


clean:
	rm -f $(OBJECTS) $(DEPENDS) $(TARGET)
