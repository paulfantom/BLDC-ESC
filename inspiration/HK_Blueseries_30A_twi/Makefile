PROJECTNAME	= hk_blueseries_30a_twi
PROJECTSRC	=	\
core/main.c		\
core/interrupts.c	\
core/maintenance.c	\
core/run.c		\
core/start.c		\
core/util.c		\
interface.c		\
twi_slave/twi_slave.c

MCU		= atmega8
AVRDUDE_MCU	= m8
AVRDUDE_PROGID	= stk200
AVRDUDE_PORT	= /dev/parport0

INC		= -I/usr/avr/include
LIBS		= -L/usr/i686-pc-linux-gnu/avr/lib
OPTLEVEL	= s

CFLAGS		=	-I. $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
			-mcall-prologues			\
			-fpack-struct -fshort-enums             \
			-funsigned-bitfields -funsigned-char    \
			-Wall -Wstrict-prototypes               \
			-Wa,-ahlms=$(firstword                  \
			$(filter %.lst, $(<:.c=.lst)))

CPPFLAGS	=	-fno-exceptions               \
			-Wa,-ahlms=$(firstword         \
			$(filter %.lst, $(<:.cpp=.lst))\
			$(filter %.lst, $(<:.cc=.lst)) \
			$(filter %.lst, $(<:.C=.lst)))

ASMFLAGS	=	-I. $(INC) -mmcu=$(MCU)        \
			-x assembler-with-cpp            \
			-Wa,-gstabs,-ahlms=$(firstword   \
			$(<:.S=.lst) $(<.s=.lst))

LDFLAGS=-Wl,-Map,$(TRG).map -mmcu=$(MCU) $(LIBS) 
	 
CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
SIZE=avr-size
AVRDUDE=avrdude
RM=rm -f -v

TRG=$(PROJECTNAME).out
DUMPTRG=$(PROJECTNAME).s
HEXROMTRG=$(PROJECTNAME).hex 
HEXTRG=$(HEXROMTRG) $(PROJECTNAME).ee.hex

CPPFILES=$(filter %.cpp, $(PROJECTSRC))
CCFILES=$(filter %.cc, $(PROJECTSRC))
BIGCFILES=$(filter %.C, $(PROJECTSRC))
CFILES=$(filter %.c, $(PROJECTSRC))
ASMFILES=$(filter %.S, $(PROJECTSRC))
OBJDEPS=$(CFILES:.c=.o)    \
	$(CPPFILES:.cpp=.o)\
	$(BIGCFILES:.C=.o) \
	$(CCFILES:.cc=.o)  \
	$(ASMFILES:.S=.o)
LST=$(filter %.lst, $(OBJDEPS:.o=.lst))
GENASMFILES=$(filter %.s, $(OBJDEPS:.o=.s)) 
BACKUPS=*~

.SUFFIXES : .c .cc .cpp .C .o .out .s .S .hex .ee.hex .h .hh .hpp

all: $(TRG)

disasm: $(DUMPTRG) stats

stats: $(TRG)
	$(OBJDUMP) -h $(TRG)
	$(SIZE) $(TRG) 

hex: $(HEXTRG)

writeflash: hex
	$(AVRDUDE) -c $(AVRDUDE_PROGID) -p $(AVRDUDE_MCU) -P $(AVRDUDE_PORT) \
	-e -U flash:w:$(HEXROMTRG) -y

install: writeflash

clean:
	$(RM) $(TRG)
	$(RM) $(TRG).map
	$(RM) $(DUMPTRG)
	$(RM) $(OBJDEPS)
	$(RM) $(LST)
	$(RM) $(GENASMFILES)
	$(RM) $(HEXTRG)
	$(RM) $(BACKUPS)

$(DUMPTRG): $(TRG)
	$(OBJDUMP) -S  $< > $@


$(TRG): $(OBJDEPS) 
	$(CC) $(LDFLAGS) -o $(TRG) $(OBJDEPS)
	$(SIZE) $(TRG) 
	
%.s: %.c
	$(CC) -S $(CFLAGS) $< -o $@

%.s: %.S
	$(CC) -S $(ASMFLAGS) $< > $@

.cpp.s .cc.s .C.s :
	$(CC) -S $(CFLAGS) $(CPPFLAGS) $< -o $@

.c.o: 
	$(CC) $(CFLAGS) -c $< -o $@

.cc.o .cpp.o .C.o :
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

.S.o :
	$(CC) $(ASMFLAGS) -c $< -o $@

.out.hex:
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

.out.ee.hex:
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@
