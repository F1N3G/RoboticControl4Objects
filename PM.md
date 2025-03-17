# **Controlul Robotic: Modele Matematice și Implementare MATLAB**

## **1. Problemele de Control Robotic**
### a) **Controlul cinematicii**
- Determinarea poziției și orientării robotului.
- Problemele de **cinematică directă** (FK) și **cinematică inversă** (IK).

### b) **Controlul dinamicii**
- Modelează forțele și momentele necesare pentru mișcare.

### c) **Controlul traiectoriei**
- Generarea și urmărirea traiectoriilor optime.

### d) **Controlul robust și adaptiv**
- Se adaptează la perturbări externe.

### e) **Controlul cooperativ**
- Coordonarea roboților colaborativi.

### f) **Controlul percepției și planificării mișcării**
- Folosit în robotică mobilă.

---

## **2. Modele Matematice**
### a) **Modele de cinematică**
#### **Transformări omogene și matrice D-H**
\[
T = T_1 \cdot T_2 \cdot \dots \cdot T_n
\]

### b) **Modele dinamice**
#### **Ecuațiile Euler-Lagrange:**
\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}} \right) - \frac{\partial L}{\partial q} = \tau
\]

#### **Ecuațiile Newton-Euler:**
- Modelează forțele și momentele asupra fiecărei legături.

### c) **Modele de Control**
#### **Control PID:**
\[
 u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{d e(t)}{dt}
\]

#### **Control optim (LQR):**
\[
J = \int_0^\infty (x^T Q x + u^T R u) dt
\]

#### **Control predictiv (MPC)**
- Prevede stările viitoare și optimizează în timp real.

---

## **3. Aplicații Industriale și Medicale**

### **Industrie:**
✅ Optimizarea traiectoriei roboților industriali.  
✅ Controlul forței și evitarea coliziunilor.  
✅ Roboți colaborativi (cobots) și logistică autonomă.

### **Robotica Medicală:**
✅ Chirurgie asistată robotic.  
✅ Exoschelete și proteze inteligente.  
✅ Control neuronal optimizat.

---

## **4. Implementare MATLAB**
### **a) Cinematică Directă (FK)**
```matlab
clc; clear; close all;
L1 = 1; L2 = 0.5;
theta1 = pi/4; theta2 = pi/6;
x = L1*cos(theta1) + L2*cos(theta1 + theta2);
y = L1*sin(theta1) + L2*sin(theta1 + theta2);
fprintf('Poziția efectorului final: (%.2f, %.2f)\n', x, y);
figure; hold on; axis equal;
plot([0, L1*cos(theta1)], [0, L1*sin(theta1)], 'r', 'LineWidth', 2);
plot([L1*cos(theta1), x], [L1*sin(theta1), y], 'b', 'LineWidth', 2);
scatter([0, L1*cos(theta1), x], [0, L1*sin(theta1), y], 'ko', 'filled');
xlabel('X'); ylabel('Y'); title('Robot 2-DOF - Cinematica Directă');
grid on;
```

### **b) Control PID pentru Dinamică**
```matlab
clc; clear; close all;
m1 = 1; m2 = 0.5; L1 = 1; L2 = 0.5; g = 9.81;
dt = 0.01; T = 5; t = 0:dt:T;
theta1_d = pi/3; theta2_d = pi/4;
theta1 = 0; theta2 = 0;
theta1_dot = 0; theta2_dot = 0;
theta1_int = 0; theta2_int = 0;
Kp = 100; Ki = 10; Kd = 20;
theta1_hist = zeros(size(t));
theta2_hist = zeros(size(t));
for i = 1:length(t)
    e1 = theta1_d - theta1;
    e2 = theta2_d - theta2;
    theta1_int = theta1_int + e1 * dt;
    theta2_int = theta2_int + e2 * dt;
    tau1 = Kp * e1 + Ki * theta1_int + Kd * (0 - theta1_dot);
    tau2 = Kp * e2 + Ki * theta2_int + Kd * (0 - theta2_dot);
    theta1_ddot = (tau1 - m2*L2*g*cos(theta1 + theta2)) / (m1*L1^2 + m2*L2^2);
    theta2_ddot = (tau2 - m2*L2*g*cos(theta2)) / (m1*L1^2 + m2*L2^2);
    theta1_dot = theta1_dot + theta1_ddot * dt;
    theta2_dot = theta2_dot + theta2_ddot * dt;
    theta1 = theta1 + theta1_dot * dt;
    theta2 = theta2 + theta2_dot * dt;
    theta1_hist(i) = theta1;
    theta2_hist(i) = theta2;
end
figure;
subplot(2,1,1); plot(t, theta1_hist, 'r', t, theta2_hist, 'b');
xlabel('Timp [s]'); ylabel('Unghi [rad]'); legend('\theta_1', '\theta_2');
title('Control PID pentru Robot 2-DOF');
subplot(2,1,2); plot(t, theta1_d*ones(size(t)), 'r--', t, theta2_d*ones(size(t)), 'b--');
xlabel('Timp [s]'); ylabel('Setpoint [rad]'); legend('\theta_1 target', '\theta_2 target');
grid on;
```

---

## **5. Concluzie**
✅ **Cinematica directă** – Calcularea poziției efectorului final.  
✅ **Dinamică** – Modelarea mișcării robotului.  
✅ **Control PID** – Stabilizarea robotului la poziția dorită.
