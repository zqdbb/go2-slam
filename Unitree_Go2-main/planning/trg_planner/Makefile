# Check for sudo availability
SUDO := $(shell if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then echo "sudo"; else echo ""; fi)

# Common directories
BUILD_DIR := cpp/trg_planner/build
SOURCE_DIR := cpp/trg_planner

# Dependencies for the project
DEPS := gcc g++ build-essential libeigen3-dev python3-pip python3-dev cmake git

# Display ASCII art at the beginning of the build process
ascii_art:
	@echo "============================================================"
	@echo "████████╗██████╗  ██████╗                                   "
	@echo "╚══██╔══╝██╔══██╗██╔════╝                                   "
	@echo "   ██║   ██████╔╝██║  ███╗█████╗                            "
	@echo "   ██║   ██╔══██╗██║   ██║╚════╝                            "
	@echo "   ██║   ██║  ██║╚██████╔╝                                  "
	@echo "   ╚═╝   ╚═╝  ╚═╝ ╚═════╝                                   "
	@echo "                                                            "
	@echo "██████╗ ██╗      █████╗ ███╗   ██╗███╗   ██╗███████╗██████╗ "
	@echo "██╔══██╗██║     ██╔══██╗████╗  ██║████╗  ██║██╔════╝██╔══██╗"
	@echo "██████╔╝██║     ███████║██╔██╗ ██║██╔██╗ ██║█████╗  ██████╔╝"
	@echo "██╔═══╝ ██║     ██╔══██║██║╚██╗██║██║╚██╗██║██╔══╝  ██╔══██╗"
	@echo "██║     ███████╗██║  ██║██║ ╚████║██║ ╚████║███████╗██║  ██║"
	@echo "╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝"
	@echo "============================================================"

# Install project dependencies
deps:
	@echo "Installing dependencies..."
	@$(SUDO) apt update -y
	@$(SUDO) apt install -y $(DEPS)

# Clean the project
clean_cpp:
	@rm -rf $(BUILD_DIR)

# Helper function to create build and run cmake
build_cpp:
	@mkdir -p $(BUILD_DIR)
	@cmake -B$(BUILD_DIR) $(SOURCE_DIR) -DCMAKE_BUILD_TYPE=Release
	@cmake --build $(BUILD_DIR) -j$(nproc --all)

# Install the project
install_cpp:
	@$(SUDO) cmake --install $(BUILD_DIR)

# Full build and install
cppinstall: clean_cpp build_cpp install_cpp ascii_art
	@echo "Enjoy Our TRG-planner!"
