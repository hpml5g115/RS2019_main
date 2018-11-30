#-------------- basic settings ---------------

#SRC = class.cpp myfunc.cpp move.cpp grouping.cpp nonstop_v2.cpp

OBJS = class.o myfunc.o move.o grouping.o main.o

TARGET = nonstop

CFLAGS = -Wall -O3 `pkg-config --libs opencv` -L/home/pi/rp_libs -lrplidar -lpthread -lwiringPi

LDFLAGS = `pkg-config --cflags opencv` -I/home/pi/rp_libs/include

CC = g++

#-------------- compile and link ---------------
$(TARGET): $(OBJS)
	$(CC) -ggdb $(LDFLAGS) -o $(TARGET) $(OBJS) $(CFLAGS)

.cpp.o:
	$(CC) -c $< $(LDFLAGS)
#-------------- others ------------------------

.PHONY: clean

clean:
	rm $(TARGET) $(OBJS)
