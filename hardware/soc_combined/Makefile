.PHONY: check-util check-util-ci patch_sapphire firmware

SCRIPTS_DIR := $(shell git rev-parse --show-toplevel 2>/dev/null)/scripts
ifeq ($(SCRIPTS_DIR),/scripts)
SCRIPTS_DIR := ../../scripts
endif

SYM_PREFIX := EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol

check-util:
        python $(SCRIPTS_DIR)/check_ti60_utilisation.py --report-dir work_syn

check-util-ci:
        python $(SCRIPTS_DIR)/check_ti60_utilisation.py --report-dir work_syn --missing-ok

firmware:
        $(MAKE) -C firmware clean
        $(MAKE) -C firmware

patch_sapphire: $(SYM_PREFIX)0.bin
        python3 $(SCRIPTS_DIR)/patch_sapphire_init.py sapphire.v \
                $(SYM_PREFIX)0.bin \
                $(SYM_PREFIX)1.bin \
                $(SYM_PREFIX)2.bin \
                $(SYM_PREFIX)3.bin

$(SYM_PREFIX)0.bin: firmware
