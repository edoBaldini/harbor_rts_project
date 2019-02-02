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
CFLAGS = -I. -Wall

#--------------------------------------------------- 
# LIBS links the libraries exploited 
#---------------------------------------------------
LIBS = -lpthread  -lm 
#--------------------------------------------------- 
# Dependencies 
#---------------------------------------------------
$(MAIN): $(MAIN).o ptask.o
		$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).c ptask.o `allegro-config --libs` $(LIBS)

ptask.o: ptask.c
		$(CC) -c ptask.c

clean: 
		rm -f *.o 
		rm -f core