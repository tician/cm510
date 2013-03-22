#~~~~~~~~~~~~~~~~~~~~ Output File Name ~~~~~~~~~~~~~~~~~~~~
MAIN_OUT = HaViMo2_APP_CM510

#~~~~~~~~~~~~~~~~~~~~ Source Files ~~~~~~~~~~~~~~~~~~~~
MAIN_SRCC = \
    ../../easy-functions/src/cm510.c \
    main.c

MAIN_SRCCPP = 

MAIN_SRCASM = 

#~~~~~~~~~~~~~~~~~~~~ Include Directories ~~~~~~~~~~~~~~~~~~~~
INCLUDE_DIRS = -I. -I../../easy-functions/inc 

#~~~~~~~~~~~~~~~~~~~~ Library Directories ~~~~~~~~~~~~~~~~~~~~
LIBRARY_DIRS = -L. 

#~~~~~~~~~~~~~~~~~~~~ User Libraries ~~~~~~~~~~~~~~~~~~~~
USER_LIBS = 
#USER_LIBS = -lzigbee -lserial -ldynamixel

#~~~~~~~~~~~~~~~~~~~~ MCU name ~~~~~~~~~~~~~~~~~~~~
#MCU = atmega168
#MCU = attiny13
#MCU = t13
MCU = atmega2561

#~~~~~~~~~~~~~~~~~~~~ Processor frequency ~~~~~~~~~~~~~~~~~~~~
#     This will define a symbol, F_CPU, in all source code files equal to the 
#     processor frequency. You can then use this symbol in your source code to 
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
#     Typical values are:
#         F_CPU =  1000000
#         F_CPU =  1843200
#         F_CPU =  2000000
#         F_CPU =  3686400
#         F_CPU =  4000000
#         F_CPU =  7372800
#         F_CPU =  8000000
#         F_CPU = 11059200
#         F_CPU = 14745600
F_CPU = 16000000
#         F_CPU = 18432000
#         F_CPU = 20000000
#F_CPU = 9600000


#~~~~~~~~~~~~~~~~~~~~ Optimization level ~~~~~~~~~~~~~~~~~~~~
#    Can be [0, 1, 2, 3, s]. 
#    0 = turn off optimization. s = optimize for size.
#    (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
#OPT = s
OPT = 2

#~~~~~~~~~~~~~~~~~~~~ C Standard level ~~~~~~~~~~~~~~~~~~~~
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99
CDEFS = -DF_CPU=$(F_CPU)UL
CPPDEFS = -DF_CPU=$(F_CPU)UL
ADEFS = -DF_CPU=$(F_CPU)

#~~~~~~~~~~~~~~~~~~~~ Compiler Options C ~~~~~~~~~~~~~~~~~~~~
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -mmcu=$(MCU) -I. 
#CFLAGS += -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -Wall
CFLAGS += -Wstrict-prototypes
#CFLAGS += -mshort-calls
#CFLAGS += -fno-unit-at-a-time
#CFLAGS += -Wundef
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wsign-compare
CFLAGS += -Wa,-adhlns=$(<:%.c=%.lst)
#CFLAGS += $(patsubst %,-I%,$(INCLUDE_DIRS))
CFLAGS += $(CSTANDARD)

#~~~~~~~~~~~~~~~~~~~~ Compiler Options C++ ~~~~~~~~~~~~~~~~~~~~
CPPFLAGS = -mmcu=$(MCU) -I. -x c++ 
#CPPFLAGS += -g$(DEBUG)
CPPFLAGS += $(CPPDEFS)
CPPFLAGS += -O$(OPT)
CPPFLAGS += -funsigned-char
CPPFLAGS += -funsigned-bitfields
CPPFLAGS += -fpack-struct
CPPFLAGS += -fshort-enums
CPPFLAGS += -fno-exceptions
CPPFLAGS += -Wall
CPPFLAGS += -Wundef
#CPPFLAGS += -mshort-calls
#CPPFLAGS += -fno-unit-at-a-time
#CPPFLAGS += -Wstrict-prototypes
#CPPFLAGS += -Wunreachable-code
#CPPFLAGS += -Wsign-compare
CPPFLAGS += -Wa,-adhlns=$(<:%.cpp=%.lst)
#CPPFLAGS += $(patsubst %,-I%,$(INCLUDE_DIRS))
#CPPFLAGS += $(CSTANDARD)

#~~~~~~~~~~~~~~~~~~~~ Compiler Options Assembly ~~~~~~~~~~~~~~~~~~~~
ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS) -Wa,-adhlns=$(<:%.S=%.lst),-gstabs,--listing-cont-lines=100

#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
PRINTF_LIB = 
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB = 
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)

MATH_LIB = -lm

#---------------- External Memory Options ----------------

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--section-start,.data=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =

#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$@.map,--cref
LDFLAGS += $(EXTMEMOPTS)
#LDFLAGS += $(patsubst %,-L%,$(LIBRARY_DIRS))
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB) $(USER_LIBS)
#LDFLAGS += -Wl,--section-start=.bootloader=0x1C00
#LDFLAGS += -Wl,--section-start=.bootsupport=0x1D00
#LDFLAGS += -Wl,--section-start=.text=0x1C00
#LDFLAGS += -T linker_script.x

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MMD -MP -MF .dep/$(@F).d

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TCHAIN_PREFIX=avr-

CC = $(TCHAIN_PREFIX)gcc
CFLAGS += $(GENDEPFLAGS) $(INCLUDE_DIRS) 

CXX = $(TCHAIN_PREFIX)g++
CXXFLAGS += $(GENDEPFLAGS) $(INCLUDE_DIRS) 

AS = $(TCHAIN_PREFIX)gcc
ASFLAGS += $(GENDEPFLAGS)

LD = $(TCHAIN_PREFIX)gcc
LDFLAGS += $(INCLUDE_DIRS) $(LIBRARY_DIRS) 

OBJCP = $(TCHAIN_PREFIX)objcopy

OBJDUMP = $(TCHAIN_PREFIX)objdump

AR = $(TCHAIN_PREFIX)ar
ARFLAGS = rcs
#ARFLAGS = cr

NM =  $(TCHAIN_PREFIX)nm

SHELL = sh
WINSHELL = cmd
SIZE = $(TCHAIN_PREFIX)size
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp


MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_CPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:


#~~~~~~~~~~~~~~~~~~~~ Linked Files ~~~~~~~~~~~~~~~~~~~~
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_HEX = $(MAIN_OUT).hex
MAIN_OUT_EEP = $(MAIN_OUT).eep
MAIN_OUT_LSS = $(MAIN_OUT).lss
MAIN_OUT_SYM = $(MAIN_OUT).sym

#~~~~~~~~~~~~~~~~~~~~ Object Files ~~~~~~~~~~~~~~~~~~~~
MAIN_OBJS = \
    $(MAIN_SRCC:%.c=%.o) \
    $(MAIN_SRCCPP:%.cpp=%.o) \
    $(MAIN_SRCASM:%.s=%.o)

#~~~~~~~~~~~~~~~~~~~~ all ~~~~~~~~~~~~~~~~~~~~
all: begin gccversion sizebefore build sizeafter end

#~~~~~~~~~~~~~~~~~~~~ build ~~~~~~~~~~~~~~~~~~~~
build: \
    $(MAIN_OUT_ELF) \
    $(MAIN_OUT_HEX) \
    $(MAIN_OUT_EEP) \
    $(MAIN_OUT_LSS) \
    $(MAIN_OUT_SYM)

#~~~~~~~~~~~~~~~~~~~~ Eye candy ~~~~~~~~~~~~~~~~~~~~
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo

sizebefore:
	@if test -f $(MAIN_OUT_ELF); then echo; \
	echo $(MSG_SIZE_BEFORE);                \
	$(SIZE) $(MAIN_OUT_ELF);                \
	$(SIZE) --target=ihex $(MAIN_OUT_HEX);  \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(MAIN_OUT_ELF); then echo; \
	echo $(MSG_SIZE_AFTER);                 \
	$(SIZE) $(MAIN_OUT_ELF);                \
	$(SIZE) --target=ihex $(MAIN_OUT_HEX);  \
	2>/dev/null; echo; fi

gccversion : 
	@$(CC) --version

#~~~~~~~~~~~~~~~~~~~~ Creating Output Files ~~~~~~~~~~~~~~~~~~~~
# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCP) -O ihex -R .eeprom -R .fuse -R .lock $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCP) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings \
	-O ihex $< $@ || exit 0

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -z $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

#~~~~~~~~~~~~~~~~~~~~ Linking ~~~~~~~~~~~~~~~~~~~~
# Link: create ELF output file from object files.
.SECONDARY : $(MAIN_OUT_ELF)
.PRECIOUS : $(MAIN_OBJS)
%.elf: $(MAIN_OBJS)
	@echo
	@echo $(MSG_LINKING) $@
	$(LD) $(CFLAGS) $^ --output $@ $(LDFLAGS)

#~~~~~~~~~~~~~~~~~~~~ Compiling ~~~~~~~~~~~~~~~~~~~~
# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) $(CFLAGS) -c $< -o $@ 

# Compile: create object files from C++ source files.
%.o : %.cpp
	@echo
	@echo $(MSG_COMPILING_CPP) $<
	$(CXX) $(CPPFLAGS) -c $< -o $@ 

# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(CFLAGS) $< -o $@

# Compile: create assembler files from C++ source files.
%.s : %.cpp
	$(CXX) -S $(CPPFLAGS) $< -o $@

# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(AS) -c $(ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@ 


# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

#~~~~~~~~~~~~~~~~~~~~ clean ~~~~~~~~~~~~~~~~~~~~
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(MAIN_OUT_HEX)
	$(REMOVE) $(MAIN_OUT_ELF)
	$(REMOVE) $(MAIN_OUT_ELF).map
	$(REMOVE) $(MAIN_OUT_LSS)
	$(REMOVE) $(MAIN_OUT_EEP)
	$(REMOVE) $(MAIN_OUT_SYM)
	$(REMOVEDIR) .dep
	$(REMOVE) $(MAIN_OBJS)
	$(REMOVE) $(MAIN_OBJS:%.o=%.lst)

#~~~~~~~~~~~~~~~~~~~~ backup ~~~~~~~~~~~~~~~~~~~~
backup: clean
	tar cJvf ../$(MAIN_OUT)_`date +"%Y-%m-%d_%H%M"`.tar.xz *
