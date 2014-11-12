# Prepare and build all flicker sources

APPS=$(shell find apps/* -type d)
UNAME:=$(shell uname)
ifeq ($(shell uname), Linux)
	MF = Makefile.linux
#else ...
endif

all: kmod/flicker.ko $(APPS)

.PHONY: force kmod/flicker.ko

kmod: kmod/flicker.ko

apps: $(APPS)

kmod/flicker.ko: kmod/Makefile
	@echo "Building flicker kernel module"
	@make -C kmod

kmod/Makefile:
	@echo "Symlinking kernel module makefile to $(MF)"
	@-cd kmod && ln -s $(MF) Makefile

$(APPS): force
	@echo "Building application '$@'"
	@make -C $@

run:
	@if [ "x$(APP)" = "x" ]; then \
	    echo "Please provide a valid $$APP parameter"; \
	else \
	    echo "Running application $(APP)"; \
	    cd apps && sudo ./go.sh $(APP)/pal.bin; \
	fi

