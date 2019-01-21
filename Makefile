#---------------------------------------------------
# Target file to be compiled by default
#---------------------------------------------------
MAIN = radar
#---------------------------------------------------
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc
#---------------------------------------------------
# CFLAGS will be the options passed to the compiler
#---------------------------------------------------
CFLAGS = -Wall -lpthread -lrt
#---------------------------------------------------
# Dependencies
#---------------------------------------------------
$(MAIN): $(MAIN).o ptask.o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o ptask.o `allegro-config --libs`
$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c
ptask.o: ptask.c
	$(CC) -c ptask.c