AS = gpasm
CC = sdcc
CFLAGS= -c -mpic16 -p18f97j60  -o$@ 
LD = sdcc
LDFLAGS= -mpic16 -p18f97j60 -L/usr/local/lib/pic16 -llibio18f97j60.lib \
         -llibdev18f97j60.lib -llibc18f.a
AR = ar
RM = rm

OBJECTS= Objects/LCDBlocking.o

SDCC_HEADERS=/usr/local/share/sdcc/include/string.h \
   /usr/local/share/sdcc/include/stdlib.h \
   /usr/local/share/sdcc/include/stdio.h \
   /usr/local/share/sdcc/include/stddef.h \
   /usr/local/share/sdcc/include/stdarg.h 

SDCC_PIC16_HEADERS=/usr/local/share/sdcc/include/pic16/pic18f97j60.h

TCPIP_HEADERS=   Include/TCPIP_Stack/ETH97J60.h \
   Include/TCPIP_Stack/LCDBlocking.h 

APP_HEADERS=Include/GenericTypeDefs.h \
   Include/Compiler.h \
   Include/HardwareProfile.h

reveil : Objects/reveil.o $(OBJECTS)
	$(LD) $(LDFLAGS) Objects/reveil.o $(OBJECTS)

findfreq : Objects/findfreq.o $(OBJECTS)
	$(LD) $(LDFLAGS) Objects/findfreq.o $(OBJECTS)

Objects/reveil.o : reveil.c $(SDCC_HEADERS) $(SDCC_PIC16_HEADERS) \
   $(APP_HEADERS) $(TCPIP_HEADERS)
	$(CC) $(CFLAGS) reveil.c

Objects/melanie.o : melanie.c $(SDCC_HEADERS) $(SDCC_PIC16_HEADERS) \
   $(APP_HEADERS) $(TCPIP_HEADERS)
	$(CC) $(CFLAGS) melanie.c

Objects/findfreq.o : findfreq.c $(SDCC_HEADERS) $(SDCC_PIC16_HEADERS) \
   $(APP_HEADERS) $(TCPIP_HEADERS)
	$(CC) $(CFLAGS) findfreq.c


Objects/LCDBlocking.o : TCPIP_Stack/LCDBlocking.c $(SDCC_HEADERS)  \
   $(SDCC_PIC16_HEADERS) $(APP_HEADERS) $(TCPIP_HEADERS)
	$(CC) -c -mpic16 -p18f97j60  -o"Objects/LCDBlocking.o" \
              -L/usr/local/lib/pic16  TCPIP_Stack/LCDBlocking.c

Objects/Tick.o : TCPIP_Stack/Tick.c  $(SDCC_HEADERS)  \
   $(SDCC_PIC16_HEADERS) $(APP_HEADERS) $(TCPIP_HEADERS)
	$(CC) -c -mpic16 -p18f97j60  -o"Objects/Tick.o" \
              -L/usr/local/lib/pic16  TCPIP_Stack/Tick.c

clean : 
	$(RM) -rf *.o
