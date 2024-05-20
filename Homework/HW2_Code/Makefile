CC=gcc

all: waiter signaler

waiter: waiter.c
	$(CC) $(CFLAGS) waiter.c -o waiter -lrt -lpthread

signaler: signaler.c
	$(CC) $(CFLAGS) signaler.c -o signaler -lrt -lpthread

clean:
	rm -f waiter signaler
