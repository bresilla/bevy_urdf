SHELL := /bin/bash

PROJECT_NAME := $(shell sed -n '/^[[:space:]]*[^#\[[:space:]]/p' PROJECT | head -1 | tr -d '[:space:]')
PROJECT_VERSION := $(shell sed -n '/^[[:space:]]*[^#\[[:space:]]/p' PROJECT | sed -n '2p' | tr -d '[:space:]')
ifeq ($(PROJECT_NAME),)
    $(error Error: PROJECT file not found or invalid)
endif

TOP_DIR := $(CURDIR)
CARGO := cargo
BIN := urdf-viewer
# DISPLAY pins which X server receives the window (matches the Nvidia GL
# display when running inside WSL / multi-X setups). Override if you need
# `:0` or similar: `make run DISPLAY=:0`.
DISPLAY ?= :1
# Wrapper that forwards GPU/display access. `nixVulkan` = Bevy/wgpu path.
# Override with `make run RUN_WITH=nixGL` or `RUN_WITH=` for native.
RUN_WITH ?= nixVulkan

# Optional robot key forwarded to the binary (matches a RobotEntry key in
# src/bin/urdf_viewer.rs). Example: `make run ROBOT=ur5`.
ROBOT ?=

$(info ------------------------------------------)
$(info Project: $(PROJECT_NAME) v$(PROJECT_VERSION))
$(info ------------------------------------------)

.PHONY: build b compile c run r release check fmt bench clean help h

build:
	@$(CARGO) build --bin $(BIN)

b: build

compile:
	@$(CARGO) clean
	@$(MAKE) build

c: compile

run:
	@DISPLAY=$(DISPLAY) $(RUN_WITH) $(CARGO) run --bin $(BIN) -- $(ROBOT)

r: run

release:
	@DISPLAY=$(DISPLAY) $(RUN_WITH) $(CARGO) run --release --bin $(BIN) -- $(ROBOT)

check:
	@$(CARGO) check --bin $(BIN)

fmt:
	@$(CARGO) fmt --all

bench:
	@$(CARGO) bench

clean:
	@$(CARGO) clean

help:
	@echo
	@echo "Usage: make [target]"
	@echo
	@echo "Available targets:"
	@echo "  build        Build the $(BIN) binary"
	@echo "  compile      Clean and rebuild"
	@echo "  run          Run: DISPLAY=$(DISPLAY) $(RUN_WITH) cargo run --bin $(BIN)"
	@echo "  release      Run in release mode"
	@echo "  check        Run cargo check on the binary"
	@echo "  fmt          Format the workspace"
	@echo "  bench        Run benchmarks"
	@echo "  clean        Remove Cargo build artifacts"
	@echo
	@echo "Examples:"
	@echo "  make run"
	@echo "  make run ROBOT=ur5            # load a specific catalog entry at startup"
	@echo "  make run DISPLAY=:0           # target a different X server"
	@echo "  make run RUN_WITH=nixGL       # OpenGL wrapper instead of Vulkan"
	@echo "  make run RUN_WITH=            # no wrapper (native run)"
	@echo

h: help
