# ================================================================
#  Docker Registry Publishing Targets
#  Include this in your main Makefile or run: make -f mk/publish.mk
# ================================================================

# Include infra for any infrastructure targets if needed
include $(dir $(abspath $(lastword $(MAKEFILE_LIST))))/infra.mk

REGISTRY := ghcr.io/comrob
IMAGE_NAME := lili-sam
LOCAL_IMAGE := lili_jazzy_prod
TAG ?= jazzy
LATEST ?= false

.PHONY: help-publish tag push publish pull clean-images

help-publish:
	@echo "Docker Registry Publishing Commands"
	@echo "===================================="
	@echo "  make -f mk/publish.mk tag              - Tag image for registry (default: jazzy)"
	@echo "  make -f mk/publish.mk tag TAG=latest   - Tag with custom tag"
	@echo "  make -f mk/publish.mk push             - Push to GitHub registry"
	@echo "  make -f mk/publish.mk publish          - Tag + Push (one command)"
	@echo "  make -f mk/publish.mk pull             - Pull from registry"
	@echo "  make -f mk/publish.mk clean-images     - Remove local images"
	@echo ""
	@echo "Examples:"
	@echo "  make -f mk/publish.mk publish TAG=v1.0.0"
	@echo "  make -f mk/publish.mk publish TAG=latest LATEST=true"

# Tag the local image for registry
tag:
	@echo "Tagging $(LOCAL_IMAGE) as $(REGISTRY)/$(IMAGE_NAME):$(TAG)..."
	docker tag $(LOCAL_IMAGE) $(REGISTRY)/$(IMAGE_NAME):$(TAG)
	@echo "✓ Tagged successfully"
	@docker images | grep $(IMAGE_NAME) | head -5

# Push to GitHub Container Registry
push:
	@echo "Pushing $(REGISTRY)/$(IMAGE_NAME):$(TAG)..."
	@echo "Make sure you're logged in: docker login ghcr.io"
	docker push $(REGISTRY)/$(IMAGE_NAME):$(TAG)
	@echo "✓ Pushed successfully"

# Tag as latest (optional)
tag-latest:
	@echo "Tagging $(REGISTRY)/$(IMAGE_NAME):$(TAG) as latest..."
	docker tag $(REGISTRY)/$(IMAGE_NAME):$(TAG) $(REGISTRY)/$(IMAGE_NAME):latest
	docker push $(REGISTRY)/$(IMAGE_NAME):latest
	@echo "✓ Latest tag updated"

# Convenience: Tag + Push in one command
publish: tag push
	@echo "✓ Published $(REGISTRY)/$(IMAGE_NAME):$(TAG)"
	@if [ "$(LATEST)" = "true" ]; then $(MAKE) -f mk/publish.mk tag-latest; fi

# Pull from registry
pull:
	@echo "Pulling $(REGISTRY)/$(IMAGE_NAME):$(TAG)..."
	docker pull $(REGISTRY)/$(IMAGE_NAME):$(TAG)
	@echo "✓ Pulled successfully"

# List local images
list-images:
	@echo "Local LILI images:"
	@docker images | grep -E "$(IMAGE_NAME)|lili_jazzy"

# Remove local images
clean-images:
	@echo "Removing local LILI images..."
	@docker rmi $(REGISTRY)/$(IMAGE_NAME):$(TAG) 2>/dev/null || true
	@docker rmi $(LOCAL_IMAGE) 2>/dev/null || true
	@echo "✓ Cleaned up images"

# Show registry info
info:
	@echo "Registry Configuration"
	@echo "======================"
	@echo "Registry: $(REGISTRY)"
	@echo "Image: $(IMAGE_NAME)"
	@echo "Tag: $(TAG)"
	@echo "Local image: $(LOCAL_IMAGE)"
	@echo "Full path: $(REGISTRY)/$(IMAGE_NAME):$(TAG)"
