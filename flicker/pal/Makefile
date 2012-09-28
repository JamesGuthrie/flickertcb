# Makefile made with help from http://www.hsrl.rutgers.edu/ug/make_help.html
# $@ is the name of the file to be made.
# $? is the names of the changed dependents.
# $< the name of the related file that caused the action.
# $* the prefix shared by target and dependent files.

CC=gcc
RM=rm -f
AS=as
AR=ar
LD=ld
OBJCOPY=objcopy

CFLAGS := -Wall -Werror -O2 -std=gnu99 -m32 -march=i686
CFLAGS += -ffreestanding -nostdlib 
CFLAGS += -fno-builtin -fno-common -fno-strict-aliasing
CFLAGS += -fomit-frame-pointer
CFLAGS += -I$(CURDIR)/../include
CFLAGS += -msoft-float
CFLAGS += -fno-jump-tables
CFLAGS += -fno-stack-protector

CFLAGS += -DnotPERFCRIT
CFLAGS += -DSERIAL_BASE_OVERRIDE=0x5050
CFLAGS += -DMIN_LOG_LEVEL=1

AFLAGS += -D__ASSEMBLY__

# necessary to support gcc 64-bit "built-ins" in 32-bit build
# e.g., undefined reference errors re: __udivdi3 or __umoddi3
GCC_LIBS := $(shell $(CC) -m32 --print-file-name=libgcc.a)

LDFLAGS	+= -melf_i386

TARGET := $(CURDIR)/pal
TARGET_LDS := $(CURDIR)/pal.ld


C_SRCS := $(wildcard $(CURDIR)/*.c)
S_SRCS := $(wildcard $(CURDIR)/*.S)
HDRS   := $(wildcard $(CURDIR)/*.h)

OBJS := $(S_SRCS:.S=.o) $(C_SRCS:.c=.o)

BUILD_DEPS := $(CURDIR)/Makefile

all: $(TARGET).bin

$(TARGET).elf: $(OBJS) $(TARGET_LDS)
	$(LD) $(LDFLAGS) -T $(TARGET_LDS) -N $(OBJS) $(GCC_LIBS) \
	    -o $(@D)/$(@F)

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

clean:
	$(RM) $(TARGET).bin $(TARGET).elf $(OBJS)

%.o : %.c $(HDRS) $(BUILD_DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

%.o : %.S $(HDRS) $(BUILD_DEPS)
	$(CC) $(AFLAGS) $(CFLAGS) -c $< -o $@

%.i : %.c $(HDRS) $(BUILD_DEPS)
	$(CPP) $(CFLAGS) $< -o $@

# -std=gnu{89,99} gets confused by # as an end-of-line comment marker
%.s : %.S $(HDRS)  $(BUILD_DEPS)
	$(CPP) $(AFLAGS) $< -o $@
