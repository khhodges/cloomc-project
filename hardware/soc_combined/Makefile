.PHONY: check-util check-util-ci

SCRIPTS_DIR := $(shell git rev-parse --show-toplevel 2>/dev/null)/scripts
ifeq ($(SCRIPTS_DIR),/scripts)
SCRIPTS_DIR := ../../scripts
endif

check-util:
	python $(SCRIPTS_DIR)/check_ti60_utilisation.py --report-dir work_syn

check-util-ci:
	python $(SCRIPTS_DIR)/check_ti60_utilisation.py --report-dir work_syn --missing-ok
