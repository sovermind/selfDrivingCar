IDIR = ../include
CC = g++
CFLAGS = -c -Wall -std=c++11

all: JerryPro1 ;

JerryPro1: main.o gridNode.o aStarSearch.o myPQ.o
	$(CC) -o JerryPro1 main.o gridNode.o aStarSearch.o myPQ.o

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

gridNode.o: gridNode.cpp gridNode.h
	$(CC) $(CFLAGS) gridNode.cpp

aStarSearch.o: aStarSearch.cpp aStarSearch.h
	$(CC) $(CFLAGS) aStarSearch.cpp

myPQ.o: myPQ.cpp myPQ.h
	$(CC) $(CFLAGS) myPQ.cpp

.PHONY: clean

clean:
	rm -f JerryPro1 *.o