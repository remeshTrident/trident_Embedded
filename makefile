CC=gcc
CFLAGS = -lpthread -lrt
INLUDE = .

mcs :main.o mcs_ctrlr.o util.o bit.o mcs_socket.o radio_commands.o
	$(CC) $(CFLAGS) -o mcs main.o mcs_ctrlr.o util.o bit.o mcs_socket.o radio_commands.o

main.o: main.c main.h
	$(CC) -I$(INCLUDE) $(CFLAGS) -c main.c

mcs_ctrlr.o: mcs_ctrlr.c mcs_ctrlr.h config.h
	$(CC) -I$(INCLUDE) $(CFLAGS) -c mcs_ctrlr.c

util.o: util.c util.h
	$(CC) -I$(INCLUDE) $(CFLAGS) -c util.c

bit.o: bit.c bit.h
	$(CC) -I$(INCLUDE) $(CFLAGS) -c bit.c

mcs_socket.o: mcs_socket.c mcs_socket.h
	$(CC) -I$(INCLUDE) $(CFLAGS) -c mcs_socket.c

radio_commands.o: radio_commands.c radio_commands.h
	$(CC) -I$(INCLUDE) $(CFLAGS) -c radio_commands.c

clean: 
	-rm mcs main.o mcs_ctrlr.o util.o bit.o mcs_socket.o radio_commands.o
