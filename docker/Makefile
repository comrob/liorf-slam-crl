# ================================================================
#  LILI-SAM - Main Makefile (Dispatcher)
#  Delegates to specialized .mk files for better organization
# ================================================================

.PHONY: help

help:
	@echo "LILI-SAM Docker Environment"
	@echo "========================================"
	@echo ""
	@echo "Infrastructure (mk/infra.mk):"
	@echo "  make up          - Start containers in background"
	@echo "  make down        - Stop and remove containers"
	@echo "  make image       - Build Docker Image (Uses Cache - Fast)"
	@echo "  make reimage     - Force Rebuild Docker Image (No Cache - Slow)"
	@echo "  make clean       - Remove containers and network"
	@echo "  make prune       - System-wide cleanup"
	@echo "  make stop        - Stop containers"
	@echo ""
	@echo "Development (mk/dev.mk):"
	@echo "  make build       - Compile the ROS 2 package (Colcon)"
	@echo "  make rebuild     - Clean build cache and rebuild"
	@echo "  make slam        - Run SLAM (launches Rviz + node)"
	@echo "  make prod        - Run production SLAM (pre-built)"
	@echo "  make play        - Play the configured bag file"
	@echo "  make shell       - Enter the dev container shell"
	@echo "  make bag_shell   - Enter the bag_player shell"
	@echo ""
	@echo "Publishing (mk/publish.mk):"
	@echo "  make tag         - Tag image for registry (TAG=jazzy)"
	@echo "  make push        - Push to GitHub registry (TAG=jazzy)"
	@echo "  make publish     - Tag + Push (TAG=jazzy)"
	@echo "  make info        - Show registry configuration"
	@echo ""
	@echo "Examples:"
	@echo "  make build && make slam"
	@echo "  make publish TAG=v1.0.0"
	@echo "  make -f mk/dev.mk shell"
	@echo ""

# ================================================================
#  Dispatcher Targets - delegate to specialized makefiles
# ================================================================

# Infrastructure
up down clean prune clean-build reimage image stop:
	@$(MAKE) -f mk/infra.mk $@

# Development
build rebuild slam prod play shell bag_shell:
	@$(MAKE) -f mk/dev.mk $@

# Publishing
tag push publish info:
	@$(MAKE) -f mk/publish.mk $@
