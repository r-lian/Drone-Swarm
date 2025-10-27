# Complete Drone Swarm Setup Guide

## 📋 Prerequisites

This setup guide will help you configure the complete drone swarm development environment using WSL2.

## 🚀 Quick Start

### Step 1: Install Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Essential build tools
sudo apt install -y build-essential cmake git curl wget

# C++ development
sudo apt install -y gcc g++ make pkg-config

# Additional dependencies
sudo apt install -y libssl-dev libcurl4-openssl-dev
```

### Step 2: Clone External Dependencies

**IMPORTANT**: Do NOT add these to your Git repository. They are external dependencies.

```bash
# Clone PX4-Autopilot (outside your Drone-Swarm directory)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.0

# Install PX4 dependencies
bash ./Tools/setup/ubuntu.sh

# Clone MAVSDK (outside your Drone-Swarm directory)  
cd ~
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git checkout v1.4.12

# Build MAVSDK
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### Step 3: Install Simulation Environment

```bash
# Install Gazebo Garden
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-garden
```

### Step 4: Build Your Drone Project

```bash
cd ~/Drone-Swarm/drone-main

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build
make -j$(nproc)
```

## 🎯 How to Use

### Starting the Simulator

```bash
# In Terminal 1: Start PX4
cd ~/PX4-Autopilot
make px4_sitl_default gazebo

# In Terminal 2 (optional): Start QGroundControl
qgroundcontrol
```

### Running Your Programs

All programs require a `--connection_url` parameter. For simulation, use `udp://:14540`.

**Basic Operations:**
```bash
# Takeoff
./build/takeoff --connection_url udp://:14540

# Telemetry
./build/telemetry --connection_url udp://:14540 --time_to_exit_in_ms 10000 --sampling_frequency_in_hz 1.0
```

**Flight Patterns:**
```bash
# Square pattern
./build/action_goto_gpsay_square --connection_url udp://:14540 --side_length 10.0 --speed 5.0

# 3D Cube
./build/cube udp://:14540

# Velocity-based square
./build/velocity_timeout_square --connection_url udp://:14540 --side_length 10.0 --speed 5.0
```

**Multi-Drone (Swarm):**
```bash
# Terminal 1: Start 3 drones in simulator
cd ~/PX4-Autopilot
./Tools/gazebo_sitl_multiple_run.sh -s "iris:3"

# Terminal 2: Start leader
./build/leader --connection_url udp://:14540 &

# Terminal 3: Start followers
./build/partner --connection_url udp://:14541 --north 10.0 --east 0.0 --down -5.0 --yaw 0.0 --speed 5.0 &
```

## 📁 Project Structure

```
Drone-Swarm/
├── drone-main/          # Your main project (THIS should be in Git)
│   ├── lib/             # Core library components
│   ├── test_flights/    # Test flight programs
│   ├── utilities/       # Utility programs
│   └── build/          # Build output (IGNORED by Git)
├── PX4-Autopilot/       # External dependency (IGNORED)
├── MAVSDK/             # External dependency (IGNORED)
└── .gitignore          # Tells Git what to ignore
```

## ⚠️ Important Notes

1. **Do NOT commit PX4-Autopilot or MAVSDK** - These are large external repositories (~500MB+ each)
2. **Do NOT commit build/ directories** - These contain compiled binaries
3. **Clone dependencies separately** outside your project directory
4. **Use .gitignore** to automatically exclude these from version control

## 🔧 Troubleshooting

### MAVSDK Not Found
```bash
sudo ldconfig
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
```

### Clean Build
```bash
cd drone-main
rm -rf build
mkdir build && cd build
cmake .. && make -j$(nproc)
```

### WSL Display Issues
If running GUI applications in WSL, you may need X11 forwarding or WSLg support.

## 📚 Additional Resources

- [PX4 Documentation](https://docs.px4.io/)
- [MAVSDK Documentation](https://mavsdk.mavlink.io/)
- [Gazebo Documentation](https://gazebosim.org/docs)

