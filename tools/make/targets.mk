# Part of CrazyFlie's Makefile
# Copyright (c) 2009, Arnaud Taffanel
# Common targets with verbose support

.PHONY: clean clean_p mrproper

ifndef VERBOSE
.SILENT:
endif

target = @$(if $(QUIET), ,echo $($1_COMMAND$(VERBOSE)) ); @$($1_COMMAND)

.c.o:
	echo "\tBuilding " $@
	$(CC) $(CFLAGS) -c $< -o $(BIN)/$@

.cpp.o: 
	echo "\tBuilding " $@
	$(CPP) $(CPPFLAGS) -g -c -std=c++11 $< -o $(BIN)/$@

.S.o:
	echo "\tBuilding " $@
	$(CC) $(CSFLAGS) -c $< -o $(BIN)/$@

$(PROG).elf: $(OBJ)
	echo "\tBuilding " $@
	$(LD) $(LDFLAGS) $(foreach o,$(OBJ),$(BIN)/$(o)) -lm -o $@

$(PROG).hex: $(PROG).elf
	echo "\tBuilding " $@
	$(OBJCOPY) $< -O ihex $@

$(PROG).bin: $(PROG).elf
	echo "\tBuilding " $@
	$(OBJCOPY) $< -O binary --pad-to 0 $@

$(PROG).dfu: $(PROG).bin
	echo "\tBuilding " $@
	$(PYTHON2) tools/make/dfu-convert.py -b $(LOAD_ADDRESS):$< $@

.s.o:
	echo "\tBuilding " $@
	$(AS) $(ASFLAGS) $< -o $(BIN)/$@

clean_o: clean_version
	$(foreach o,$(OBJ),$(BIN)/$(o))

clean:
	rm -f cf*.elf cf*.hex cf*.bin cf*.dfu cf*.map $(BIN)/dep/*.d $(BIN)/*.o input_for_motor_ports.txt simulation.bin

mrproper: clean
	rm -f *~ hal/src/*~ hal/interface/*~ tasks/src/*~ tasks/inc/*~ utils/src/*~ utils/inc/*~ tools/make/*~; rm -rf bin/dep/*.d $(BIN)/*.a $(BIN)/vendor/*.o
