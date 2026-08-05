# ================================================================
#  Docker Infrastructure Management
#  up, down, clean, prune, reimage, image building
# ================================================================

HOST_RENDER_GID := $(shell stat -c "%g" /dev/dri/renderD128 2>/dev/null || getent group render | cut -d: -f3 2>/dev/null || echo 110)

DOCKER_DIR := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../)

# 2. Inject RENDER_GID dynamically as an inline environment variable into Docker Compose 
COMPOSE := RENDER_GID=$(HOST_RENDER_GID) docker compose -f $(DOCKER_DIR)/docker-compose.yaml --env-file $(DOCKER_DIR)/.env
Q := @
.PHONY: up down clean prune clean-build reimage stop image rebuild

up:
	$(Q)$(COMPOSE) up -d lili_dev bag_player

down:
	$(Q)$(COMPOSE) down lili_dev bag_player

clean:
	@echo "Removing project containers and network..."
	$(Q)$(COMPOSE) down --remove-orphans lili_dev bag_player lili_run

prune:
	@echo "Pruning all stopped containers system-wide..."
	$(Q)docker container prune -f

clean-build:
	@echo "Cleaning ros lili package build cache (preserving .empty files)..."
	$(Q)find $(DOCKER_DIR)/cache/build -mindepth 1 ! -name ".empty" -exec rm -rf {} +
	$(Q)find $(DOCKER_DIR)/cache/install -mindepth 1 ! -name ".empty" -exec rm -rf {} +
	$(Q)find $(DOCKER_DIR)/cache/log -mindepth 1 ! -name ".empty" -exec rm -rf {} +
	@echo "Cache cleaned."

# [NEW] Builds the image using existing cache (Fast)
image:
	@echo "Building Docker Image (Cached)..."
	$(Q)$(COMPOSE) build

reimage:
	@echo "⚠️  WARNING: You are about to rebuild the Docker image with --no-cache."
	@echo "   This will recompile GTSAM and heavy dependencies (10+ minutes)."
	@echo -n "   Are you sure you want to proceed? [y/N] " && read ans && [ $${ans:-N} = y ]
	@echo "Rebuilding from scratch..."
	$(Q)$(COMPOSE) build --no-cache
	$(Q)$(COMPOSE) up -d

stop:
	$(Q)$(COMPOSE) down
