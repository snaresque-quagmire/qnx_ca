CFLAGS=-Wall -pedantic
LIBS=-lm

func4: hello.c
	$(CC) $(CFLAGS) -o func4 hello.c $(LIBS)


clean:
	rm -f func4
