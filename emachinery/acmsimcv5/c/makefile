
# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard *.c)

CFLAGS = -I. -L.

%.o : %.c
	gcc -c $(CFLAGS) $< -o $@

main: $(SRC)
	gcc -o $@ $^ $(CFLAGS)

# https://stackoverflow.com/questions/170467/makefiles-compile-all-c-files-at-once
# https://stackoverflow.com/questions/3932895/makefile-aliases/3933012#3933012

