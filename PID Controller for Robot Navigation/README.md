# PID Controller for Robot Navigation

This repository contains a C++ implementation of a PID (Proportional-Integral-Derivative) controller for robot navigation, coupled with plotting capabilities using `matplotlibcpp`.

## Table of Contents

- [PID Controller for Robot Navigation](#pid-controller-for-robot-navigation)
  - [Table of Contents](#table-of-contents)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
    - [macOS](#macos)
    - [Ubuntu](#ubuntu)
  - [Building the Project](#building-the-project)
  - [Running the Project](#running-the-project)
  - [PID Controller Theory](#pid-controller-theory)
    - [Effect of Tuning Parameters](#effect-of-tuning-parameters)

## Dependencies

- CMake
- Python (with Matplotlib and NumPy)
- A C++ compiler

## Installation

### macOS

1. **Install Homebrew** (if not already installed):

    ```bash
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    ```

2. **Install Dependencies**:

    ```bash
    brew install cmake python
    pip3 install matplotlib numpy
    ```

### Ubuntu

1. **Update Repositories**:

    ```bash
    sudo apt update
    ```

2. **Install Dependencies**:

    ```bash
    sudo apt install cmake python3 python3-pip g++
    pip3 install matplotlib numpy
    ```

## Building the Project

```bash
mkdir build
cd build
cmake ..
make
```

## Running the Project

After building, execute the resulting binary:

```bash
./PIDController
```

## PID Controller Theory

A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error by adjusting the process control inputs. The PID controller calculation involves three separate constant parameters: the proportional, integral, and derivative values, denoted as Kp, Ki, and Kd.

### Effect of Tuning Parameters

- **Kp (Proportional Gain)**:
  - Increasing Kp will make the system respond faster to errors but might lead to overshooting.
  - Decreasing Kp will make the system respond slower to errors, reducing overshoot but potentially leading to steady-state error.

- **Ki (Integral Gain)**:
  - Increasing Ki will allow the system to eliminate steady-state errors, but might introduce overshoot and oscillations.
  - Decreasing Ki reduces the chance of overshoot but might lead to steady-state errors.

- **Kd (Derivative Gain)**:
  - Increasing Kd will "predict" error behavior and counteract it, reducing overshooting and oscillations.
  - Decreasing Kd will make the controller less sensitive to fast changes, which could be detrimental if the system needs to react quickly to changes.

Please note that tuning a PID controller is often an iterative process, and the behaviors described above might vary based on the specific system being controlled. You can play around with paramters in the `main.cpp` file to see how the controller behaves in different scenarios.
