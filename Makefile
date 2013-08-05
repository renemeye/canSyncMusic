CC=gcc
CFLAGS=$(shell xeno-config --skin=native --cflags) -g
LFLAGS=$(shell xeno-config --skin=native --ldflags) -lrtdm 


canSync: canSync.o
	$(CC) -o canSync canSync.o $(LFLAGS)

canSync.o: canSync.c
	$(CC) -c canSync.c $(CFLAGS)




clean:
	  rm -rf *~ *.o canWrite canSync
