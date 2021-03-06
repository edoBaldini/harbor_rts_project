#--------------------------------------------------- 
# Target file to be compiled by default 
#---------------------------------------------------
MAIN = harbor

#--------------------------------------------------- 
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc

#--------------------------------------------------- 
# CFLAGS will be the options passed to the compiler 
#---------------------------------------------------
CFLAGS = -I. -Wall -g

#--------------------------------------------------- 
# LIBS links the libraries exploited 
#---------------------------------------------------
LIBS = -lpthread -lm `allegro-config --libs`

#--------------------------------------------------- 
# OBJS objects
#---------------------------------------------------
OBJS = ptask.o common.o user.o ship.o

#--------------------------------------------------- 
# Dependencies 
#---------------------------------------------------
$(MAIN): $(MAIN).o ptask.o common.o user.o ship.o
		$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).c $(OBJS) $(LIBS)

ptask.o: ptask.c
		$(CC) -c ptask.c

common.o: common.c
		$(CC) -c common.c

ship.o: ship.c
		$(CC) -c ship.c

user.o: user.c
		$(CC) -c user.c

clean: 
		rm -f *.o 
		rm -f core