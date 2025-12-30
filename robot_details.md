# Robot Details: Differential Swerve Platform

## Overview

This robot is a holonomic mobile platform featuring 4 independent drive modules. Based on the URDF and mechanical design, each module is a **Differential Swerve** unit.

In this system, the steering of the module is **coupled** to the wheel rotation. There is no dedicated motor for the steering axis. Instead, the steering motion is induced mechanically by the speed difference between the two drive wheels.

## Joint Configuration

The robot has **12 Revolute Joints** in total.

### Actuated Joints (Motors)
These are the joints we directly control (8 total, 2 per module).
- **FLDriveL, FLDriveR** (Front Left Module)
- **FRDriveL, FRDriveR** (Front Right Module)
- **BLDriveL, BLDriveR** (Back Left Module)
- **BRDriveL, BRDriveR** (Back Right Module)

### Passive Joints (Feedback Only)
These joints are **not actuated**. They rotate freely (physically driven by the differential action of the drive wheels). We use their encoders to measure the current steering angle of the module.
- **FLSteer**
- **FRSteer**
- **BLSteer**
- **BRSteer**

## Kinematics Derivation

### 1. Geometric Parameters

**Robot Footprint (Steer Pivots):**
derived from URDF `*Steer` origins:
- **Front-Left (FL)**: $(-0.257, 0.252)$
- **Front-Right (FR)**: $(0.256, 0.252)$
- **Back-Left (BL)**: $(-0.257, -0.261)$
- **Back-Right (BR)**: $(0.256, -0.261)$
- **Track/Wheelbase**: $\approx 0.513 \, m \times 0.513 \, m$

**Module Geometry:**
- **Wheel Separation ($b$)**: $\approx 0.028 \, m$ (Distance between left and right wheel contact points).
- **Wheel Radius ($r$)**: $\approx 0.0575 \, m$ (115mm diameter).

### 2. Module Kinematics (Differential Swerve)

The motion of the module is determined entirely by the angular velocities of its two wheels: $\omega_L$ and $\omega_R$.

**Forward Kinematics (Sensing):**
To determine what the module is doing based on wheel encoders:
1.  **Module Speed ($v_{mod}$)**:
    $$ v_{mod} = \frac{r (\omega_L + \omega_R)}{2} $$
2.  **Steering Rate ($\Omega_{mod}$)** (Rate of change of the passive Steer joint):
    $$ \Omega_{mod} = \dot{\theta}_{steer} = \frac{r (\omega_R - \omega_L)}{b} $$

**Inverse Kinematics (Control):**
To achieve a desired module state ($v_{target}$, $\Omega_{target}$):
Since we cannot apply torque to the Steer joint directly, we must induce rotation by spinning the wheels at different speeds.
$$ \omega_L = \frac{v_{target} - \Omega_{target} \cdot (b/2)}{r} $$
$$ \omega_R = \frac{v_{target} + \Omega_{target} \cdot (b/2)}{r} $$

*Note: $\Omega_{target}$ is usually the output of a PID controller trying to align the actual measured `theta_steer` to the desired angle.*

### 3. Robot (Chassis) Kinematics

To move the chassis at velocities $V_x, V_y, \omega_{robot}$:

1.  **Compute Module Vector**: For each module $i$ at $(x_i, y_i)$:
    $$ \vec{v}_{i} = \begin{bmatrix} V_x - \omega_{robot}y_i \\ V_y + \omega_{robot}x_i \end{bmatrix} $$

2.  **Convert to Polar**:
    $$ v_{target, i} = ||\vec{v}_i|| $$
    $$ \theta_{target, i} = \text{atan2}(v_{iy}, v_{ix}) $$

3.  **Optimize (Swerve Logic)**:
    Compare $\theta_{target, i}$ with current feedback $\theta_{meas, i}$. If error $> 90^\circ$, flip velocity and add $180^\circ$ to target.

4.  **Steering Control Loop**:
    Calculate steering error: $e_{\theta} = \theta_{target, i} - \theta_{meas, i}$.
    Compute required rotation rate: $\Omega_{cmd} = K_p \cdot e_{\theta}$.

5.  **Wheel Velocity Command**:
    Apply Inverse Module Kinematics using $v_{target, i}$ and $\Omega_{cmd}$ to find $\omega_L, \omega_R$.
