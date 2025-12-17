CC = g++

CPPUTEST_HOME = /usr/include/CppUTest

CFLAGS += -include $(CPPUTEST_HOME)/MemoryLeakDetectorMallocMacros.h
CXXFLAGS += -include $(CPPUTEST_HOME)/MemoryLeakDetectorNewMacros.h
CXXFLAGS += -O0 -g3 -Wall -Wextra

TARGET = a.out

SRCS = *.cpp

OBJS = $(SRCS:.cpp=.o)

INCDIR = -I../

LIBDIR =

LIBS =

LD_LIBRARIES = -L$(CPPUTEST_HOME) -L../ -lCppUTest -lCppUTestExt

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBDIR) $(LD_LIBRARIES) $(LIBS)

$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) $(CXXFLAGS) $(INCDIR) -c $(SRCS)

all: clean $(OBJS) $(TARGET)

clean:
	-rm -f $(OBJS) $(TARGET) *.d
