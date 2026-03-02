
.DEFAULT_GOAL := help

%:
	@$(MAKE) -C docker $@