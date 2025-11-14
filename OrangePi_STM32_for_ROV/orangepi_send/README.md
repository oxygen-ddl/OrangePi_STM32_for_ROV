# Orange Pi to STM32 PWM Control Over UDP

## 1. Project Overview

This project implements a host-side application designed to run on an Orange Pi (or other Linux-based host). It sends PWM control commands and heartbeat packets to an STM32 microcontroller over UDP. The primary goal is to establish a real-time communication link for robotics or motor control applications.

The application is written in C++ and utilizes a custom communication protocol for sending data frames. It is built using CMake and is intended for a POSIX-compliant environment.

### Key Features:

- **UDP Communication**: A lightweight `UdpSender` class handles sending and receiving UDP packets.
- **Custom Protocol**: A well-defined protocol (`protocol_pack`) is used to structure data frames, including PWM commands and heartbeats. It ensures data integrity with CRC checks.
- **High-Frequency Control**: The application is designed to send PWM commands at a configurable frequency (e.g., 50Hz).
- **Heartbeat Mechanism**: A regular heartbeat is sent to monitor the connection status, and it includes logic for Round-Trip Time (RTT) calculation.
- **Cross-Platform Build**: Uses CMake for easy compilation on various Linux distributions.

## 2. Environment Configuration

To build and run this project, you will need a Linux environment with the following tools and libraries installed. These instructions are based on a Debian-based distribution like Ubuntu.

### System Requirements:

- **Operating System**: Linux (e.g., Ubuntu 18.04+, Debian 10+, or the Linux distribution running on your Orange Pi).
- **Compiler**: A C++17 compatible compiler, such as GCC 7+ or Clang 5+.
- **Build Tools**: `cmake` (version 3.10 or higher) and `make`.
- **Standard Libraries**: `pthreads` for threading support. This is typically included with the compiler.

### Installation Steps:

1.  **Install Build Essentials**:
    Open a terminal and run the following command to install the necessary build tools and compiler:

    ```bash
    sudo apt-get update
    sudo apt-get install build-essential cmake
    ```

2.  **Verify Installation**:
    Check the versions of the installed tools to ensure they meet the requirements:

    ```bash
    g++ --version
    cmake --version
    ```

## 3. Build Instructions

Follow these steps to compile the project from the source code.

1.  **Clone the Repository**:
    If you haven't already, clone the project to your local machine.

2.  **Create a Build Directory**:
    It is best practice to create a separate directory for the build files.

    ```bash
    cd /path/to/orangepi_send
    mkdir build
    cd build
    ```

3.  **Run CMake to Configure the Project**:
    CMake will generate the necessary Makefiles for compilation.

    ```bash
    cmake ..
    ```

4.  **Compile the Project**:
    Use `make` to build the executable.

    ```bash
    make
    ```

    If the build is successful, you will find the executable `pwm_udp_sender` inside the `build` directory.

## 4. Usage

The `pwm_udp_sender` executable can be run from the terminal. It accepts optional command-line arguments to configure the target device's IP address and port, as well as the frequency of control and heartbeat messages.

### Command-Line Arguments: