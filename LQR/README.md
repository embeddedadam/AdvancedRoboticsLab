# LQR Stabilization Project

This project implements a Linear Quadratic Regulator (LQR) for stabilizing a robot. The LQR algorithm is widely used in control theory to design a controller that regulates the state of a system to achieve a desired performance while minimizing a certain cost.

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

## LQR - Linear Quadratic Regulator

The LQR algorithm is designed to determine the input to a system that minimizes a cost defined in terms of the system's state and control input. The cost is:

 ```math
 J = ∫ (x^T Q x + u^T R u) dt
 ```

 Where:

- `x` is the state of the system.
- `u` is the control input.
- `Q` and `R` are weight matrices chosen by the designer to prioritize state and control costs.

The objective of LQR is to determine the control law `u` that minimizes this cost. For linear systems, the control law is a linear feedback:

```math
u = -K x
```

Where `K` is the feedback gain matrix determined by LQR.

## Riccati Equation

To determine the matrix `K`, we solve the continuous-time algebraic Riccati equation. This equation arises from the condition of optimality and plays a crucial role in finding the optimal control law. In this project, the Riccati equation is solved iteratively.

## Methods to Solve the Riccati Equation

1. **Analytical Methods**:
   - For some simple systems, the Riccati equation can be solved analytically.
   - This is rarely used in practice for complex systems.

2. **Numerical Integration**:
   - Differential Riccati equations can be integrated using methods like Euler's method and Runge-Kutta.

3. **DARE (Discrete-time Algebraic Riccati Equation) Solvers**:
   - Specialized algorithms exist for discrete-time systems.

4. **CARE (Continuous-time Algebraic Riccati Equation) Solvers**:
   - Algorithms specifically for continuous-time systems.

5. **Iterative Methods**:
   - Includes Newton's method and gradient methods.

6. **Schur Method**:
   - Reduces the Riccati problem to finding a stable invariant subspace of an associated Hamiltonian matrix.

7. **Eigendecomposition**:
   - Solutions can be found via eigenvalues and eigenvectors of associated matrices.

8. **Software and Control Toolboxes**:
   - Tools like MATLAB and Octave offer built-in functions (`care` and `dare`).

## What's Inside `main.cpp`?

In `main.cpp`, we simulate a system's behavior under the LQR controller's influence. The system dynamics are defined, initial conditions are set, and the system's response is visualized over time. Through plotted results, you can observe the system's performance and the LQR controller's efficacy.
