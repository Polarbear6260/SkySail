# X650 PX4 + Gazebo 仿真环境
#
#   make deps     宿主机依赖（sudo，手动执行）
#   make check    离线模型自检（不需要 PX4/Gazebo）
#   make fetch    拉 PX4 v1.17.0 + gz 模型库
#   make build    编译 px4_sitl
#   make install  装入 X650 模型/世界/airframe（之后需 make build 重编）
#   make smoke    无头冒烟测试（120 s）
#   make run      带相机 + 避障世界运行

PX4_DIR ?= $(HOME)/PX4-Autopilot
export PX4_DIR

.PHONY: all deps check fetch build install smoke run run-bare clean help

all: help

help:
	@sed -n '2,10p' Makefile

deps:
	@echo "需要 root，手动执行： sudo ./scripts/00_install_host_deps.sh"

check:
	@python3 scripts/05_check_model_math.py

fetch:
	@./scripts/01_fetch_sources.sh

build:
	@./scripts/02_build_sitl.sh

install:
	@./scripts/03_install_gz_assets.sh

smoke:
	@./scripts/04_run_sitl.sh --smoke

run:
	@./scripts/04_run_sitl.sh

run-bare:
	@./scripts/04_run_sitl.sh --model x650 --world default

clean:
	@rm -rf logs
