# Linear-Quadratic-Gaussian (LQG) Control

Linear-Quadratic-Gaussian (LQG) control is a method that merges the principles of optimal control with state estimation. It's ideal for designing controllers for linear systems affected by Gaussian noise. The LQG control problem breaks down into two distinct parts: the **Linear-Quadratic Regulator (LQR)** and the **Kalman Filter**.

## 1. Linear-Quadratic Regulator (LQR)

- The LQR aims to create a feedback controller that minimizes a specific quadratic cost function. This cost function often signifies a balance between state deviations and the effort required for control. Given a linear system:

    ```math
    x_{t+1} = Ax_t + Bu_t + w_t
    ```

    with `x_t` being the state, `u_t` the control input, `w_t` the process noise, and `A` and `B` the system matrices, the objective is to reduce:

    ```math
    J = Σ (x_t' Q x_t + u_t' R u_t)
    ```

    Here, `Q` and `R` are positive definite matrices defining the weightings of the state and control efforts, respectively.

- The solution to the LQR issue provides a control law:

    ```math
    u_t = -Kx_t
    ```

    Here, `K` is the optimal state feedback gain.

## 2. Kalman Filter

- Systems in the real world frequently contain uncertainties due to process and measurement noises. The Kalman Filter is a perfect state estimator for linear systems affected by Gaussian noise.

- For system dynamics:

    ```math
    x_{t+1} = Ax_t + Bu_t + w_t
    y_t = Cx_t + v_t
    ```

    Where `y_t` is the output or measurement, `C` is the output matrix, and `v_t` is the measurement noise, the Kalman Filter offers an estimate `x̂_t` of the state.

## 3. LQG Control

- By combining the LQR and Kalman Filter, the control input is:

    ```math
    u_t = -Kx̂_t
    ```

- Thus, instead of relying on the true state `x_t` (which might be unavailable due to noise and other uncertainties), the LQG controller uses the estimated state `x̂_t` from the Kalman Filter to apply the optimal control.

### Benefits of LQG Control

- Provides a methodical approach to design controllers for linear systems with noise.
- Fuses the principles of optimal control (LQR) with optimal state estimation (Kalman Filter).

### Limitations

- Works on the assumption that the system is linear and the noises are Gaussian.
- The separation principle (i.e., designing the controller and estimator separately) is valid under these assumptions but might not be the best fit for non-linear or non-Gaussian systems.

While LQG is a central concept in control theory, many real-world systems might necessitate more intricate techniques, especially when dealing with nonlinearities or non-Gaussian disturbances. Nonetheless, understanding LQG sets a firm foundation for exploring more intricate control strategies.

## Applications of Linear-Quadratic-Gaussian (LQG) Control

LQG control is versatile and can be employed in various domains, provided the systems can be approximated as linear time-invariant and are influenced by Gaussian noise. Below are some typical applications:

### 1. Aerospace

- **Aircraft Flight Control**: LQG is useful for stabilizing aircraft trajectories, especially amidst wind gusts, which can be conceptualized as Gaussian noise.
- **Satellite Attitude Control**: Ensuring satellites remain oriented correctly in space, considering various disturbances.

### 2. Robotics

- **Robot Arm Control**: For industrial robot arms, LQG can ensure precision, especially when performance is affected by disturbances and noise.
- **Autonomous Vehicle Navigation**: LQG can process sensor data to estimate a vehicle's state and control it to follow a path in noisy settings.

### 3. Electrical Systems

- **Power Grid Stabilization**: Helps maintain stability in power grids amidst fluctuations.
- **Adaptive Optics**: For large telescopes, adaptive optics, assisted by LQG, compensate for atmospheric turbulence to produce clearer images.

### 4. Mechanical Systems

- **Vibration Control**: In structures like skyscrapers or bridges, LQG can reduce vibrations caused by external factors like wind or seismic activity.
- **Suspension Systems**: In vehicles, LQG-backed active suspension systems optimize both comfort and handling.

### 5. Chemical and Biomedical Processes

- **Drug Infusion**: LQG aids in controlling drug infusion rates in biomedical contexts, especially when patient responses are unpredictable.
- **Chemical Reactor Control**: LQG can ensure optimal reactor conditions, even with disturbances in input material or external factors.

### 6. Economics

- **Inventory Control**: In supply chains, LQG can help decide order quantities based on uncertain demand and supply disturbances.

### 7. Communication Systems

- **Signal Processing**: LQG is instrumental in communication systems to extract desired signals from noisy environments.

**Note**: While LQG offers a foundational approach for these systems, real-world implementations might need extra considerations, especially when dealing with non-linearities, constraints, or non-Gaussian disturbances.
