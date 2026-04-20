% =========================================================
% ROV_PID_Init.m
% PID Controller Initialization for BlueROV2 Depth & Heading Control
%
% Run this script BEFORE opening the Simulink model.
% All workspace variables are picked up by the Simulink PID blocks.
% =========================================================

clear; clc;

fprintf('Initializing ROV PID Controller Parameters...\n');

% =========================================================
% SECTION 1: BlueROV2 Plant Parameters (von Benzon 2022)
% =========================================================
% These come from experimentally validated MSS/ROV-Simulator parameters.
% Source: von Benzon et al., JMSE 2022, Table 3 & 4.

% --- Vehicle Physical Properties ---
m   = 11.5;          % Total vehicle mass [kg]
W   = m * 9.81;      % Weight [N]
B   = 114.8;         % Buoyancy force [N]  (slightly positive buoyancy)

% --- Heave (Depth) Channel ---
% Reduced-order model: mz*w_dot + dz*w = tau_z
% mz = m + added mass in heave (Z_wdot)
Z_wdot = 5.36;       % Added mass in heave [kg]
mz = m + Z_wdot;     % Effective heave inertia [kg]

% Linear + quadratic damping in heave — use linear approximation at low speed
Z_w  = -4.8;         % Linear heave damping [N·s/m]
dz   = abs(Z_w);     % Positive damping coefficient for controller design [N·s/m]

% Net restoring force (positive buoyancy means it wants to rise)
% tau_z must overcome this to go deeper
g_z  = W - B;        % Net gravitational restoring force [N]
%   g_z < 0  => positively buoyant (needs downward thrust to dive)
%   g_z > 0  => negatively buoyant (needs upward thrust to surface)

% --- Yaw (Heading) Channel ---
% Reduced-order model: Iz*r_dot + dr*r = tau_psi
% Iz = Izz + added moment of inertia in yaw (N_rdot)
Izz    = 0.37;       % Rigid-body yaw moment of inertia [kg·m²] (heavy config)
N_rdot = 0.0;        % Added yaw inertia [kg·m²] (negligible for BlueROV2)
Iz     = Izz + N_rdot;

N_r  = -0.5;         % Linear yaw damping [N·m·s/rad]
dr   = abs(N_r);     % Positive damping coefficient [N·m·s/rad]

fprintf('  Plant parameters loaded.\n');
fprintf('  Heave: mz=%.2f kg, dz=%.2f N·s/m, g_z=%.2f N\n', mz, dz, g_z);
fprintf('  Yaw:   Iz=%.4f kg·m², dr=%.4f N·m·s/rad\n', Iz, dr);

% =========================================================
% SECTION 2: Thruster Saturation Limits
% =========================================================
% BlueROV2 T200 thruster: ~5.1 kgf forward, ~3.7 kgf reverse
% Convert to Newtons and apply a conservative limit.
tau_z_max   =  45;   % Max vertical thrust [N]
tau_z_min   = -45;
tau_psi_max =  10;   % Max yaw moment [N·m]  (two horizontal thrusters)
tau_psi_min = -10;

% =========================================================
% SECTION 3: Transfer Function Analysis
% =========================================================
% Depth TF:    Z(s)/Tau_z(s)  = 1 / (s * (mz*s + dz))
% Heading TF:  Psi(s)/Tau_psi(s) = 1 / (s * (Iz*s + dr))
%
% Both are Type-1 second-order plants. PID wraps the integrator, giving
% a Type-2 loop: zero steady-state error to step commands & ramp rejection.

s = tf('s');
P_depth   = 1 / (s * (mz*s + dz));
P_heading = 1 / (s * (Iz*s + dr));

fprintf('\n  Open-loop poles:\n');
fprintf('  Depth:   s = 0, s = %.4f\n', -dz/mz);
fprintf('  Heading: s = 0, s = %.4f\n', -dr/Iz);

% =========================================================
% SECTION 4: PID Gains — Depth Controller
% =========================================================
% Design approach: pole placement on the closed-loop characteristic equation.
% Desired depth response: critically damped, ~10s settling time.
%   Desired CL poles: s = -0.5, -0.5 (repeated), and -1.0 (integrator)
%   This gives settling time ~8–10s with no overshoot.
%
% PID + plant (1/(s(mz*s+dz))) gives 3rd-order CL.
% Matching coefficients of (s+0.5)^2*(s+1.0):

wn_z  = 0.5;         % Natural frequency for depth loop [rad/s]
zeta_z = 1.0;        % Damping ratio (critically damped)

% Gains derived from matching (s + wn_z)^2 * (s + wn_z/2):
Kp_z = mz * (2*zeta_z*wn_z + wn_z/2) - dz;
Ki_z = mz * wn_z^2 * (wn_z/2);
Kd_z = mz;

% Clamp to physically reasonable range
Kp_z = max(Kp_z, 5);
Ki_z = max(Ki_z, 0.05);
Kd_z = max(Kd_z, mz * 0.5);

fprintf('\n  Depth PID Gains:\n');
fprintf('  Kp_z = %.4f\n', Kp_z);
fprintf('  Ki_z = %.4f\n', Ki_z);
fprintf('  Kd_z = %.4f\n', Kd_z);

% =========================================================
% SECTION 5: PID Gains — Heading Controller
% =========================================================
% Desired heading response: slightly underdamped for speed, fast correction.
%   wn_psi = 1.0 rad/s, zeta_psi = 0.9 (near-critical, quick settling)
%   Settling time ~5–6s.

wn_psi   = 1.0;
zeta_psi = 0.9;

Kp_psi = Iz * (2*zeta_psi*wn_psi + wn_psi/2) - dr;
Ki_psi = Iz * wn_psi^2 * (wn_psi/2);
Kd_psi = Iz;

Kp_psi = max(Kp_psi, 0.3);
Ki_psi = max(Ki_psi, 0.01);
Kd_psi = max(Kd_psi, Iz * 0.5);

fprintf('\n  Heading PID Gains:\n');
fprintf('  Kp_psi = %.4f\n', Kp_psi);
fprintf('  Ki_psi = %.4f\n', Ki_psi);
fprintf('  Kd_psi = %.4f\n', Kd_psi);

% =========================================================
% SECTION 6: Integrator Anti-Windup Limits
% =========================================================
% Clamped to thruster saturation limits to prevent windup during large
% depth transitions (e.g., The Ekman Corkscrew: 1m -> 50m).
antiwindup_z   = tau_z_max;
antiwindup_psi = tau_psi_max;

% =========================================================
% SECTION 7: Derivative Filter Coefficient
% =========================================================
% Low-pass filter on derivative term: D(s) = Kd * N*s / (s + N)
% N = filter cutoff [rad/s]. Too high -> noisy; too low -> slow.
N_filter_z   = 10;   % Depth derivative filter [rad/s]
N_filter_psi = 15;   % Heading derivative filter [rad/s]

% =========================================================
% SECTION 8: Simulation Parameters
% =========================================================
Ts  = 0.05;          % Controller sample time [s]
T_sim = 600;         % Default simulation duration [s] (10 minutes)

% =========================================================
% SECTION 9: Verify Closed-Loop Performance (s-domain check)
% =========================================================
C_depth   = pid(Kp_z,   Ki_z,   Kd_z,   1/N_filter_z);
C_heading = pid(Kp_psi, Ki_psi, Kd_psi, 1/N_filter_psi);

CL_depth   = feedback(C_depth   * P_depth,   1);
CL_heading = feedback(C_heading * P_heading, 1);

info_z   = stepinfo(CL_depth);
info_psi = stepinfo(CL_heading);

fprintf('\n  Closed-Loop Step Response (linearized):\n');
fprintf('  Depth   — Rise: %.1fs  Settle: %.1fs  Overshoot: %.1f%%\n', ...
    info_z.RiseTime, info_z.SettlingTime, info_z.Overshoot);
fprintf('  Heading — Rise: %.1fs  Settle: %.1fs  Overshoot: %.1f%%\n', ...
    info_psi.RiseTime, info_psi.SettlingTime, info_psi.Overshoot);

% =========================================================
% SECTION 10: Export Summary to Command Window
% =========================================================
fprintf('\n=========================================================\n');
fprintf('All variables loaded into workspace. Open the Simulink\n');
fprintf('model and ensure your PID blocks reference:\n');
fprintf('  Depth PID:   Kp_z, Ki_z, Kd_z, N_filter_z\n');
fprintf('  Heading PID: Kp_psi, Ki_psi, Kd_psi, N_filter_psi\n');
fprintf('  Saturation:  tau_z_max/min, tau_psi_max/min\n');
fprintf('  Restoring:   g_z (feedforward bias on depth channel)\n');
fprintf('=========================================================\n');