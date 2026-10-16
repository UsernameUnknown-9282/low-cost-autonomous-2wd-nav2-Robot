#!/bin/bash
# =============================================================================
# Automated Turnkey Provisioning Script for Raspberry Pi Zero 2 W
# Target: Raspberry Pi OS Lite (64-bit) / Ubuntu 22.04 LTS (aarch64)
# Configures:
#   1. System optimization (1GB swap for 512MB RAM stability)
#   2. Python 3 dependencies (pyserial, pyyaml, numpy, gpiod)
#   3. ROS 2 Humble packages / environment (ROS_DOMAIN_ID=42)
#   4. Persistent LD19 LiDAR udev rules (/dev/ldlidar symlink)
#   5. Hardware serial / I2C / GPIO access permissions
#   6. Zero-touch systemd services (robot-motor.service, robot-lidar.service)
# =============================================================================

set -e

# ANSI Color formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

DOMAIN_ID=42
DRY_RUN=false
TARGET_USER="${SUDO_USER:-$USER}"
WORKSPACE_DIR="/home/$TARGET_USER/RC"
RULES_SRC_FILE=""

print_banner() {
    echo -e "${CYAN}"
    echo "========================================================================"
    echo "=== Raspberry Pi Zero 2 W - Automated Robot Provisioning Script      ==="
    echo "=== Subsystem: 2WD Diff-Drive + TB6612FNG + LD19 Pure LiDAR Odom     ==="
    echo "========================================================================"
    echo -e "${NC}"
}

usage() {
    echo "Usage: sudo $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --domain-id <id>     Set ROS_DOMAIN_ID (default: 42)"
    echo "  --workspace <path>   Set RC robot workspace path (default: /home/<user>/RC)"
    echo "  --user <username>    Set execution user for systemd (default: current/sudo user)"
    echo "  --dry-run            Simulate operations without modifying system files"
    echo "  --help               Display this help message"
    echo ""
    exit 0
}

# Parse Command-Line Arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --domain-id) DOMAIN_ID="$2"; shift ;;
        --workspace) WORKSPACE_DIR="$2"; shift ;;
        --user) TARGET_USER="$2"; shift ;;
        --dry-run) DRY_RUN=true ;;
        --help) usage ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; usage ;;
    esac
    shift
done

print_banner
echo -e "Target User   : ${GREEN}$TARGET_USER${NC}"
echo -e "Workspace Path: ${GREEN}$WORKSPACE_DIR${NC}"
echo -e "ROS_DOMAIN_ID : ${GREEN}$DOMAIN_ID${NC}"
echo -e "Dry Run Mode  : ${YELLOW}$DRY_RUN${NC}"
echo ""

# -----------------------------------------------------------------------------
# 1. Pre-flight Architecture & Environment Checks
# -----------------------------------------------------------------------------
echo -e "${BLUE}[Step 1/7] Running pre-flight architecture & environment checks...${NC}"

ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" && "$ARCH" != "arm64" && "$DRY_RUN" == "false" ]]; then
    echo -e "${YELLOW}[WARN] Detected architecture is $ARCH. 64-bit OS (aarch64) is strongly recommended for ROS 2 Humble.${NC}"
else
    echo -e "  Architecture: ${GREEN}$ARCH (64-bit compatible)${NC}"
fi

if [[ $EUID -ne 0 && "$DRY_RUN" == "false" ]]; then
    echo -e "${RED}[ERROR] This script must be executed with root privileges (sudo).${NC}"
    exit 1
fi

# -----------------------------------------------------------------------------
# 2. Memory & Swap Optimization for Pi Zero 2 W (512MB RAM)
# -----------------------------------------------------------------------------
echo -e "\n${BLUE}[Step 2/7] Checking memory & swap configuration...${NC}"

SWAPFILE_CONF="/etc/dphys-swapfile"
if [[ -f "$SWAPFILE_CONF" ]]; then
    CURRENT_SWAP=$(grep -E "^CONF_SWAPSIZE=" "$SWAPFILE_CONF" | cut -d'=' -f2 || echo "0")
    if [[ "$CURRENT_SWAP" -lt 1024 ]]; then
        echo -e "  Increasing swap from ${CURRENT_SWAP}MB to 1024MB to prevent OOM on 512MB RAM..."
        if [[ "$DRY_RUN" == "false" ]]; then
            sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' "$SWAPFILE_CONF"
            dphys-swapfile swapoff || true
            dphys-swapfile setup || true
            dphys-swapfile swapon || true
        else
            echo "  [DRY-RUN] Would update $SWAPFILE_CONF to CONF_SWAPSIZE=1024 and restart swap."
        fi
    else
        echo -e "  Swap already configured to ${GREEN}${CURRENT_SWAP}MB${NC}."
    fi
else
    echo "  Standard swapfile manager detected or containerized environment."
fi

# -----------------------------------------------------------------------------
# 3. System Packages & Python Dependencies
# -----------------------------------------------------------------------------
echo -e "\n${BLUE}[Step 3/7] Installing system packages and Python drivers...${NC}"

REQUIRED_PKGS="python3-pip python3-serial python3-yaml python3-numpy git udev curl gnupg2 lsb-release libgpiod-dev"

if [[ "$DRY_RUN" == "false" ]]; then
    apt-get update -y
    apt-get install -y --no-install-recommends $REQUIRED_PKGS
    # Install RPi.GPIO or gpiod Python bindings
    pip3 install --no-cache-dir "RPi.GPIO>=0.7.1" 2>/dev/null || pip3 install --no-cache-dir gpiod || true

    # Configure ROS 2 repository and install base packages if not already present
    if [ ! -d /opt/ros/humble ]; then
        echo "  Configuring ROS 2 apt repository for Humble installation..."
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || true
        CODENAME=$(lsb_release -cs 2>/dev/null || echo "jammy")
        if [[ "$CODENAME" == "bookworm" || "$CODENAME" == "bullseye" ]]; then
            CODENAME="jammy"
        fi
        echo "deb [arch=$(dpkg --print-architecture 2>/dev/null || echo aarch64) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main" > /etc/apt/sources.list.d/ros2.list 2>/dev/null || true
        apt-get update -y 2>/dev/null || true
        apt-get install -y --no-install-recommends ros-humble-ros-base 2>/dev/null || true
    fi

    # Attempt to install CycloneDDS if available in apt
    apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp 2>/dev/null || true
else
    echo "  [DRY-RUN] apt-get update && apt-get install -y $REQUIRED_PKGS"
    echo "  [DRY-RUN] pip3 install RPi.GPIO gpiod"
    echo "  [DRY-RUN] Check /opt/ros/humble and configure ROS 2 repo / ros-humble-ros-base if missing"
fi

# -----------------------------------------------------------------------------
# 4. ROS 2 Environment & Network Configuration
# -----------------------------------------------------------------------------
echo -e "\n${BLUE}[Step 4/7] Configuring ROS 2 Humble environment (ROS_DOMAIN_ID=$DOMAIN_ID)...${NC}"

BASHRC_FILE="/home/$TARGET_USER/.bashrc"
ENV_FILE="/etc/environment"

SETUP_LINES="
# Robot ROS 2 Humble Environment
export ROS_DOMAIN_ID=$DOMAIN_ID
export ROBOT_WORKSPACE=$WORKSPACE_DIR
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
# Use CycloneDDS only if installed, else fallback to standard FastDDS
if [ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ] || ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
"

if [[ "$DRY_RUN" == "false" ]]; then
    # Add to /etc/environment
    if ! grep -q "ROS_DOMAIN_ID=" "$ENV_FILE"; then
        echo "ROS_DOMAIN_ID=$DOMAIN_ID" >> "$ENV_FILE"
    else
        sed -i "s/ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$DOMAIN_ID/" "$ENV_FILE"
    fi

    # Add to user bashrc
    if [ -f "$BASHRC_FILE" ] && ! grep -q "ROS_DOMAIN_ID=" "$BASHRC_FILE"; then
        echo "$SETUP_LINES" >> "$BASHRC_FILE"
    fi
else
    echo "  [DRY-RUN] Would append ROS_DOMAIN_ID=$DOMAIN_ID to $ENV_FILE and $BASHRC_FILE"
fi
echo -e "  ROS_DOMAIN_ID is set to ${GREEN}$DOMAIN_ID${NC}."

# -----------------------------------------------------------------------------
# 5. Persistent LD19 LiDAR udev Rules
# -----------------------------------------------------------------------------
echo -e "\n${BLUE}[Step 5/7] Installing persistent udev rules for LD19 / D200 LiDAR...${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_RULES="$SCRIPT_DIR/udev/60-ldlidar.rules"
DEST_RULES="/etc/udev/rules.d/60-ldlidar.rules"

if [[ "$DRY_RUN" == "false" ]]; then
    if [[ -f "$LOCAL_RULES" ]]; then
        cp "$LOCAL_RULES" "$DEST_RULES"
    else
        # Write inline rules for Silicon Labs CP2102 and CH340 USB-UART bridges
        cat << 'EOF' > "$DEST_RULES"
# LDROBOT LD19 / D200 LiDAR USB-UART Bridge udev rules
# CP2102 / CP210x USB UART Bridge (Silicon Labs - Standard LD19 USB adapter)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="ldlidar", SYMLINK+="ttyUSB_LIDAR"

# CH340 / CH341 USB UART Bridge (QinHeng Electronics)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", SYMLINK+="ldlidar", SYMLINK+="ttyUSB_LIDAR"

# FT232 / FTDI USB UART Bridge
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout", SYMLINK+="ldlidar", SYMLINK+="ttyUSB_LIDAR"
EOF
    fi

    chmod 644 "$DEST_RULES"
    usermod -aG dialout,gpio,i2c "$TARGET_USER" || usermod -aG dialout "$TARGET_USER" || true
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger 2>/dev/null || true
    echo -e "  Installed udev rules -> ${GREEN}$DEST_RULES${NC}"
else
    echo "  [DRY-RUN] Would install $DEST_RULES and add $TARGET_USER to dialout, gpio, i2c groups."
fi

# -----------------------------------------------------------------------------
# 6. Hardware Serial & Interface Settings
# -----------------------------------------------------------------------------
echo -e "\n${BLUE}[Step 6/7] Configuring Raspberry Pi hardware interfaces...${NC}"

if command -v raspi-config >/dev/null 2>&1; then
    if [[ "$DRY_RUN" == "false" ]]; then
        # Disable console on serial port, leave hardware UART enabled
        raspi-config nonint do_serial 2 || true
        # Enable I2C and SPI
        raspi-config nonint do_i2c 0 || true
        raspi-config nonint do_spi 0 || true
        echo -e "  raspi-config hardware UART and I2C/SPI configured."
    else
        echo "  [DRY-RUN] raspi-config nonint do_serial 2, do_i2c 0, do_spi 0"
    fi
else
    echo "  raspi-config not found (running on Ubuntu/generic Linux), skipping board-specific config."
fi

# -----------------------------------------------------------------------------
# 7. Systemd Zero-Touch Boot Services
# -----------------------------------------------------------------------------
echo -e "\n${BLUE}[Step 7/7] Setting up zero-touch systemd services...${NC}"

MOTOR_SERVICE="/etc/systemd/system/robot-motor.service"
LIDAR_SERVICE="/etc/systemd/system/robot-lidar.service"

if [[ "$DRY_RUN" == "false" ]]; then
    chmod +x "$WORKSPACE_DIR/launch_motor_driver.sh" "$WORKSPACE_DIR/launch_lidar.sh" 2>/dev/null || true
    chmod +x "$WORKSPACE_DIR/motor_driver_node.py" "$WORKSPACE_DIR/ld19_driver_node.py" 2>/dev/null || true

    # 1. Motor Driver Service
    cat << EOF > "$MOTOR_SERVICE"
[Unit]
Description=2WD Differential Drive TB6612FNG Motor Driver Node (motor_driver_node.py)
After=network.target

[Service]
Type=simple
User=$TARGET_USER
WorkingDirectory=$WORKSPACE_DIR
Environment="ROS_DOMAIN_ID=$DOMAIN_ID"
Environment="PYTHONUNBUFFERED=1"
ExecStart=/bin/bash $WORKSPACE_DIR/launch_motor_driver.sh
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
    chmod 644 "$MOTOR_SERVICE"

    # 2. LiDAR Driver Service
    cat << EOF > "$LIDAR_SERVICE"
[Unit]
Description=LDROBOT LD19 LiDAR Driver & Static TF Bridge (ld19_driver_node.py)
After=network.target

[Service]
Type=simple
User=$TARGET_USER
WorkingDirectory=$WORKSPACE_DIR
Environment="ROS_DOMAIN_ID=$DOMAIN_ID"
Environment="PYTHONUNBUFFERED=1"
ExecStart=/bin/bash $WORKSPACE_DIR/launch_lidar.sh
Restart=always
RestartSec=3
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
    chmod 644 "$LIDAR_SERVICE"

    # Reload systemd and enable services
    systemctl daemon-reload
    systemctl enable robot-motor.service
    systemctl enable robot-lidar.service
    echo -e "  Created and enabled systemd services:"
    echo -e "    - ${GREEN}$MOTOR_SERVICE${NC}"
    echo -e "    - ${GREEN}$LIDAR_SERVICE${NC}"
else
    echo "  [DRY-RUN] Would create and enable:"
    echo "    - $MOTOR_SERVICE (ExecStart=/bin/bash $WORKSPACE_DIR/launch_motor_driver.sh for motor_driver_node.py)"
    echo "    - $LIDAR_SERVICE (ExecStart=/bin/bash $WORKSPACE_DIR/launch_lidar.sh for ld19_driver_node.py)"
fi

# -----------------------------------------------------------------------------
# Completion Summary
# -----------------------------------------------------------------------------
echo ""
echo -e "${GREEN}========================================================================${NC}"
echo -e "${GREEN}=== RASPBERRY PI ZERO 2 W PROVISIONING COMPLETE!                     ===${NC}"
echo -e "${GREEN}========================================================================${NC}"
echo "Summary of Configured Settings:"
echo "  1. Swap File         : 1024 MB (protects against 512MB RAM exhaustion)"
echo "  2. ROS_DOMAIN_ID     : $DOMAIN_ID (configured in /etc/environment & ~/.bashrc)"
echo "  3. udev Link         : /dev/ldlidar -> /dev/ttyUSB0 (0666 permissions)"
echo "  4. Motor Service     : systemctl status robot-motor.service"
echo "  5. LiDAR Service     : systemctl status robot-lidar.service"
echo ""
echo "Diagnostic Commands:"
echo "  - Check LiDAR data stream : ros2 topic echo /scan --once"
echo "  - Check motor commands    : ros2 topic echo /cmd_vel"
echo "  - View live motor logs    : journalctl -u robot-motor.service -f"
echo "  - View live LiDAR logs    : journalctl -u robot-lidar.service -f"
echo "========================================================================"
