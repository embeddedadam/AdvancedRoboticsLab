# MPC Project

This project implements a Model Predictive Controller (MPC). The MPC algorithm is widely used in control theory to design a controller that predicts and optimizes the system's future behavior to achieve a desired performance while minimizing a certain cost.

## Building the Project

### Prerequisites

- **Eigen Library**: Ensure you have Eigen installed:
  - For macOS:

    ```bash
    brew install eigen
    ```

### Compilation

1. Navigate to the project root directory.
2. Create a build directory and navigate into it:

    ```bash
    mkdir build && cd build
    ```

3. Generate the makefile and compile the project:

    ```bash
    cmake ..
    make
    ```

