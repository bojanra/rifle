#########  AVR Project Makefile Template   #########
######                                        ######
######    Copyright (C) 2003-2005,Pat Deegan, ######
######            Psychogenic Inc             ######
######          All Rights Reserved           ######
######                                        ######
###### You are free to use this code as part  ######
###### of your own applications provided      ######
###### you keep this copyright notice intact  ######
###### and acknowledge its authorship with    ######
###### the words:                             ######
######                                        ######
###### "Contains software by Pat Deegan of    ######
###### Psychogenic Inc (www.psychogenic.com)" ######
######                                        ######
###### If you use it as part of a web site    ######
###### please include a link to our site,     ######
###### http://electrons.psychogenic.com  or   ######
###### http://www.psychogenic.com             ######
######                                        ######
####################################################


##### This Makefile will make compiling Atmel AVR
##### micro controller projects simple with Linux
##### or other Unix workstations and the AVR-GCC
##### tools.
# Install following tools
# apt-get install gcc-avr avr-libc avrdude libusb-dev
#
# take care of the correct udev settings in
# /etc/udev/rules.d/80-usbprog.rules
# ATTR{idVendor}=="1781", ATTR{idProduct}=="0c9f", GROUP="plugdev", MODE="0660" # adafruit usbtiny
# ATTR{idVendor}=="03eb", ATTR{idProduct}=="c8b4", GROUP="plugdev", MODE="0660" # Alibaba programer
# edit /etc/avrdude
#
# programmer
#  id    = "usbasp-clone";
#  desc  = "Any usbasp clone with correct VID/PID";
#  type  = "usbasp";
#  connection_type = usb;
#  usbvid     = 0x03EB; # standard
#  usbpid     = 0xC8B4; # Alibaba
#  #usbvendor  = "";
#  #usbproduct = "";
#;
#
#
# read fuse bits
# low
# Now use 
# avrdude -c usbtiny -p attiny261 -v -U hfuse:r:-:h -U lfuse:r:-:h -U efuse:r:-:h
# avrdude -c usbtiny -p attiny261 -v -U lfuse:w:0xff:m -U hfuse:w:0xdc:m -U efuse:w:0xff:m
# avrdude -c usbtiny -p attiny261 -v -U hfuse:w:0x14:m
# avrdude -c usbtiny -p attiny261 -v -U lfuse:w:0xff:m
#
# avrdude -p m128 -u -U flash:w:diag.hex -U eeprom:w:eeprom.hex -U efuse:w:0xff:m -U hfuse:w:0x89:m -U lfuse:w:0x2e:m
#
#
#####
##### It supports C, C++ and Assembly source files.
#####
##### Customize the values as indicated below and :
##### make
##### make disasm
##### make stats
##### make hex
##### make writeflash
##### make gdbinit
##### or make clean
#####
##### See the http://electrons.psychogenic.com/
##### website for detailed instructions


####################################################
#####                                          #####
#####              Configuration               #####
#####                                          #####
##### Customize the values in this section for #####
##### your project. MCU, PROJECTNAME and       #####
##### PRJSRC must be setup for all projects,   #####
##### the remaining variables are only         #####
##### relevant to those needing additional     #####
##### include dirs or libraries and those      #####
##### who wish to use the avrdude programmer   #####
#####                                          #####
##### See http://electrons.psychogenic.com/    #####
##### for further details.                     #####
#####                                          #####
####################################################


#####         Target Specific Details          #####
#####     Customize these for your project     #####

# Name of target controller
# (e.g. 'at90s8515', see the available avr-gcc mmcu
# options for possible values)
MCU=attiny261

# id to use with programmer
# default: PROGRAMMER_MCU=$(MCU)
# In case the programer used, e.g avrdude, doesn't
# accept the same MCU name as avr-gcc (for example
# for ATmega8s, avr-gcc expects 'atmega8' and
# avrdude requires 'm8')
PROGRAMMER_MCU=attiny261

# Name of our project
# (use a single word, e.g. 'myproject')
PROJECTNAME=rifle

# Source files
# List C/C++/Assembly source files:
# (list all files to compile, e.g. 'a.c b.cpp as.S'):
# Use .cc, .cpp or .C suffix for C++ files, use .S
# (NOT .s !!!) for assembly source code files.
#PRJSRC=main.c myclass.cpp lowlevelstuff.S
PRJSRC=$(PROJECTNAME).c

# additional includes (e.g. -I/path/to/mydir)
#INC=-I/path/to/include
INC=

# libraries to link in (e.g. -lmylib)
LIBS=

# Optimization level,
# use s (size opt), 1, 2, 3 or 0 (off)
OPTLEVEL=s


#####      AVR Dude 'writeflash' options       #####
#####  If you are using the avrdude program
#####  (http://www.bsdhome.com/avrdude/) to write
#####  to the MCU, you can set the following config
#####  options and use 'make writeflash' to program
#####  the device.


# programmer id--check the avrdude for complete list
# of available opts.  These should include stk500,
# avr910, avrisp, bsd, pony and more.  Set this to
# one of the valid "-c PROGRAMMER-ID" values
# described in the avrdude info page.
#
AVRDUDE_PROGRAMMERID=usbtiny
#AVRDUDE_PROGRAMMERID=usbasp-clone
#AVRDUDE_PROGRAMMERID=buspirate

# port--serial or parallel port to which your
# hardware programmer is attached
#
# AVRDUDE_PORT=/dev/ttyUSB0


####################################################
#####                Config Done               #####
#####                                          #####
##### You shouldn't need to edit anything      #####
##### below to use the makefile but may wish   #####
##### to override a few of the flags           #####
##### nonetheless                              #####
#####                                          #####
####################################################


##### Flags ####

# HEXFORMAT -- format for .hex file output
HEXFORMAT=ihex

# compiler
CFLAGS=-I. $(INC) -g -mmcu=$(MCU) -O$(OPTLEVEL) \
	-fpack-struct -fshort-enums             \
	-funsigned-bitfields -funsigned-char    \
	-Wall -Wstrict-prototypes               \
	-Wa,-ahlms=$(firstword                  \
	$(filter %.lst, $(<:.c=.lst)))

# c++ specific flags
CPPFLAGS=-fno-exceptions               \
	-Wa,-ahlms=$(firstword         \
	$(filter %.lst, $(<:.cpp=.lst))\
	$(filter %.lst, $(<:.cc=.lst)) \
	$(filter %.lst, $(<:.C=.lst)))

# assembler
ASMFLAGS =-I. $(INC) -mmcu=$(MCU)        \
	-x assembler-with-cpp            \
	-Wa,-gstabs,-ahlms=$(firstword   \
		$(<:.S=.lst) $(<.s=.lst))


# linker
LDFLAGS=-Wl,-Map,$(TRG).map -mmcu=$(MCU) \
	-lm $(LIBS)

##### executables ####
CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
SIZE=avr-size
AVRDUDE=avrdude
REMOVE=rm -f

##### automatic target names ####
TRG=$(PROJECTNAME).out
DUMPTRG=$(PROJECTNAME).s

HEXROMTRG=$(PROJECTNAME).hex
HEXTRG=$(HEXROMTRG) $(PROJECTNAME).ee.hex
GDBINITFILE=gdbinit-$(PROJECTNAME)

# Define all object files.

# Start by splitting source files by type
#  C++
CPPFILES=$(filter %.cpp, $(PRJSRC))
CCFILES=$(filter %.cc, $(PRJSRC))
BIGCFILES=$(filter %.C, $(PRJSRC))
#  C
CFILES=$(filter %.c, $(PRJSRC))
#  Assembly
ASMFILES=$(filter %.S, $(PRJSRC))


# List all object files we need to create
OBJDEPS=$(CFILES:.c=.o)    \
	$(CPPFILES:.cpp=.o)\
	$(BIGCFILES:.C=.o) \
	$(CCFILES:.cc=.o)  \
	$(ASMFILES:.S=.o)

# Define all lst files.
LST=$(filter %.lst, $(OBJDEPS:.o=.lst))

# All the possible generated assembly
# files (.s files)
GENASMFILES=$(filter %.s, $(OBJDEPS:.o=.s))


.SUFFIXES : .c .cc .cpp .C .o .out .s .S \
	.hex .ee.hex .h .hh .hpp


.PHONY: writeflash clean stats gdbinit stats

# Make targets:
# all, disasm, stats, hex, writeflash/install, clean, fuses
all: $(TRG)
#all: writeflash

fuses:
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID)  \
	 -p $(PROGRAMMER_MCU) -v  \
     -U hfuse:w:0xdc:m -U lfuse:w:0xff:m -U efuse:w:0xff:m
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID)  \
	 -p $(PROGRAMMER_MCU) -v  \
   -U hfuse:r:-:h -U lfuse:r:-:h -U efuse:r:-:h


disasm: $(DUMPTRG) stats

stats: $(TRG)
	$(OBJDUMP) -h $(TRG)
	$(SIZE) $(TRG)

hex: $(HEXTRG)


writeflash: hex
	$(AVRDUDE) -c $(AVRDUDE_PROGRAMMERID)   \
	 -p $(PROGRAMMER_MCU) -e        \
	 -U flash:w:$(HEXROMTRG)

install: writeflash

$(DUMPTRG): $(TRG)
	$(OBJDUMP) -S  $< > $@


$(TRG): $(OBJDEPS)
	$(CC) $(LDFLAGS) -o $(TRG) $(OBJDEPS)


#### Generating assembly ####
# asm from C
%.s: %.c
	$(CC) -S $(CFLAGS) $< -o $@

# asm from (hand coded) asm
%.s: %.S
	$(CC) -S $(ASMFLAGS) $< > $@


# asm from C++
.cpp.s .cc.s .C.s :
	$(CC) -S $(CFLAGS) $(CPPFLAGS) $< -o $@



#### Generating object files ####
# object from C
.c.o:
	$(CC) $(CFLAGS) -c $< -o $@


# object from C++ (.cc, .cpp, .C files)
.cc.o .cpp.o .C.o :
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

# object from asm
.S.o :
	$(CC) $(ASMFLAGS) -c $< -o $@


#### Generating hex files ####
# hex files from elf
#####  Generating a gdb initialisation file    #####
.out.hex:
	$(OBJCOPY) -j .text                    \
		-j .data                       \
		-O $(HEXFORMAT) $< $@

.out.ee.hex:
	$(OBJCOPY) -j .eeprom                  \
		--change-section-lma .eeprom=0 \
		-O $(HEXFORMAT) $< $@


#####  Generating a gdb initialisation file    #####
##### Use by launching simulavr and avr-gdb:   #####
#####   avr-gdb -x gdbinit-myproject           #####
gdbinit: $(GDBINITFILE)

$(GDBINITFILE): $(TRG)
	@echo "file $(TRG)" > $(GDBINITFILE)

	@echo "target remote localhost:1212" \
		                >> $(GDBINITFILE)

	@echo "load"        >> $(GDBINITFILE)
	@echo "break main"  >> $(GDBINITFILE)
	@echo "continue"    >> $(GDBINITFILE)
	@echo
	@echo "Use 'avr-gdb -x $(GDBINITFILE)'"


#### Cleanup ####
clean:
	$(REMOVE) $(TRG) $(TRG).map $(DUMPTRG)
	$(REMOVE) $(OBJDEPS)
	$(REMOVE) $(LST) $(GDBINITFILE)
	$(REMOVE) $(GENASMFILES)
	$(REMOVE) $(HEXTRG)



#####                    EOF                   #####

