# Adaptive Control System 

This project focuses on the design and simulation of a **Feedback Linearization-based Adaptive Control** strategy for a dynamic system whose parameters are unknown or time-varying. The main objective is to ensure that the system state tracking error asymptotically converges to zero while simultaneously estimating the unknown parameters online.

The entire system is modeled and simulated within the MATLAB/Simulink environment using MATLAB Function blocks, demonstrating robust trajectory tracking performance and accurate parameter identification based on Lyapunov stability theory.

## 📌 Project Architecture and Simulink Model

The simulation layout consists of three fundamental subsystems:

1. **Trajectory Generator (`trajectory_fcn`):** Computes the desired reference trajectory $x_d = \sin(t)$ and its time derivative $\dot{x}_d = \cos(t)$ as functions of time ($t$).
2. **System Model (`model_fcn`):** Represents the actual physical plant governed by the true parameters ($a_{true} = 2.0, b_{true} = 1.5$). It computes the state derivative $\dot{x}$, which is integrated via a continuous-time integrator block to yield the current system state ($x$) for feedback.
3. **Adaptive Controller (`adaptive_control_fcn`):** Houses the control law and the online parameter adaptation laws derived via Lyapunov stability analysis. It processes the tracking error ($e$), estimated parameters ($\hat{a}, \hat{b}$), and outputs the control input ($u$).

---

## 📐 Mathematical Framework and Dynamic Model

### 1. Plant Dynamics
The true mathematical model implemented in your `model_fcn` is given by:
$$\dot{x} = a_{true} \cdot x^2 + b_{true} \cdot \cos(t) + u$$

Where the ground truth parameters are set to:
* $a_{true} = 2.0$
* $b_{true} = 1.5$

### 2. Error Dynamics
Given the reference trajectory $x_d = \sin(t)$ and its derivative $\dot{x}_d = \cos(t)$, the tracking error ($e$) is defined as:
$$e = x - x_d$$

Taking the time derivative of the tracking error yields:
e = x - x_d = a_true* x^2 + b_true*cos(t) + u -x_d

### 3. Control Law and Parameter Adaptation Rules
Since the true plant parameters ($a_{true}, b_{true}$) are assumed to be unknown to the controller, their corresponding estimated values ($\hat{a}$ and $\hat{b}$) are utilized. Utilizing the feedback linearization approach, the control input ($u$) is designed as follows:
$$u = - \hat{a} \cdot x^2 - \hat{b} \cdot \cos(t) + \dot{x}_d - k \cdot e$$

Where $k > 0$ represents a strictly positive control gain. Substituting this control law back into the error dynamics gives:
$$\dot{e} = -k \cdot e + (a_{true} - \hat{a}) \cdot x^2 + (b_{true} - \hat{b}) \cdot \cos(t)$$

Defining the parameter estimation errors as $\tilde{a} = a_{true} - \hat{a}$ and $\tilde{b} = b_{true} - \hat{b}$, the closed-loop error dynamics simplifies to:
$$\dot{e} = -k \cdot e + \tilde{a} \cdot x^2 + \tilde{b} \cdot \cos(t)$$

To guarantee the asymptotic stability of the system, a **Lyapunov Function Candidate ($V$)** is selected:
$$V(e, \tilde{a}, \tilde{b}) = \frac{1}{2} e^2 + \frac{1}{2\gamma_1} \tilde{a}^2 + \frac{1}{2\gamma_2} \tilde{b}^2$$

Where $\gamma_1, \gamma_2 > 0$ denote the adaptation gains (learning rates). To satisfy the Lyapunov condition for stability ($\dot{V} \leq -k \cdot e^2$), the **Parameter Update Laws (Adaptation Laws)** are derived as:
$$\dot{\hat{a}} = \gamma_1 \cdot e \cdot x^2$$

$$\dot{\hat{b}} = \gamma_2 \cdot e \cdot \cos(t)$$

---

## 📊 Simulation Results

The plots below illustrate the time-domain performance responses of the closed-loop system.

### 1. Tracking Error (Error)
Displays the transient and steady-state behavior of the tracking error, demonstrating asymptotic convergence to zero:

$$e = x - \sin(t)$$

<img width="1600" height="852" alt="image" src="https://github.com/user-attachments/assets/63b4de54-1fa4-40ad-8aa7-b304793826ad" />

### 2. Parameter A Estimation (A_hat vs A_true)
Shows how the online controller learns the actual plant parameter $a_{true} = 2.0$ over time and the settling behavior of $\hat{a}$.

<img width="1600" height="848" alt="image" src="https://github.com/user-attachments/assets/07d56e3d-3b67-454f-8c07-7bc90c3a56e8" />


### 3. Parameter B Estimation (B_hat vs B_true)
Illustrates the estimation process of the parameter $b_{true} = 1.5$ and the convergence of $\hat{b}$ to its true value.

<img width="1600" height="847" alt="image" src="https://github.com/user-attachments/assets/1e48e96c-1f57-4fb4-8a21-b8d6d54bc81e" />


---

## 🛠 Setup and Execution

1. Clone this repository to your local machine:
   ```bash
   git clone [https://github.com/OmerAysalX1/Adaptive_Controller.git](https://github.com/OmerAysalX1/Adaptive_Controller.git)<img width="1600" height="852" alt="error_image" src="https://github.com/user-attachments/assets/80a82bb3-a667-41e8-8cfd-f2371032129a" />
