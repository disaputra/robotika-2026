%% PROGRAM 2: INVERSE KINEMATICS ROBOT PLANAR 2-DOF (TIPE RR)
% Deskripsi: Menghitung sudut sendi (theta1, theta2) berdasarkan posisi target (x, y)

clear; clc; close all;

%% 1. PARAMETER ROBOT DAN POSISI TARGET
l1 = 1.0; % Panjang Link 1 (meter)
l2 = 0.8; % Panjang Link 2 (meter)

% Posisi Target End-Effector yang Diinginkan
x_target = 1.2;
y_target = 0.6;

fprintf('====================================================\n');
fprintf('       SIMULASI INVERSE KINEMATICS ROBOT RR         \n');
fprintf('====================================================\n');
fprintf('Parameter Robot : l1 = %.2f m, l2 = %.2f m\n', l1, l2);
fprintf('Target Posisi   : X = %.2f m, Y = %.2f m\n', x_target, y_target);
fprintf('----------------------------------------------------\n');

%% 2. PERHITUNGAN INVERSE KINEMATICS
D = (x_target^2 + y_target^2 - l1^2 - l2^2) / (2 * l1 * l2);

% Cek Ketercapaian Target (Workspace Check)
if abs(D) > 1
    error('Target (%.2f, %.2f) berada DI LUAR WORKSPACE robot!', x_target, y_target);
end

% Solusi 1: Elbow-Down
theta2_sol1 = atan2(sqrt(1 - D^2), D);
k1_1 = l1 + l2 * cos(theta2_sol1);
k2_1 = l2 * sin(theta2_sol1);
theta1_sol1 = atan2(y_target, x_target) - atan2(k2_1, k1_1);

% Solusi 2: Elbow-Up
theta2_sol2 = atan2(-sqrt(1 - D^2), D);
k1_2 = l1 + l2 * cos(theta2_sol2);
k2_2 = l2 * sin(theta2_sol2);
theta1_sol2 = atan2(y_target, x_target) - atan2(k2_2, k1_2);

%% 3. TAMPILKAN HASIL PADA KONSOL
fprintf('Solusi 1 (Elbow-Down) : theta1 = %6.2f deg, theta2 = %6.2f deg\n', ...
    rad2deg(theta1_sol1), rad2deg(theta2_sol1));
fprintf('Solusi 2 (Elbow-Up)   : theta1 = %6.2f deg, theta2 = %6.2f deg\n', ...
    rad2deg(theta1_sol2), rad2deg(theta2_sol2));
fprintf('====================================================\n\n');

%% 4. HITUNG POSISI KEDUA SOLUSI UNTUK PLOTTING
% Posisi Solusi 1
x1_s1 = l1 * cos(theta1_sol1);
y1_s1 = l1 * sin(theta1_sol1);
x_eff1 = x1_s1 + l2 * cos(theta1_sol1 + theta2_sol1);
y_eff1 = y1_s1 + l2 * sin(theta1_sol1 + theta2_sol1);

% Posisi Solusi 2
x1_s2 = l1 * cos(theta1_sol2);
y1_s2 = l1 * sin(theta1_sol2);
x_eff2 = x1_s2 + l2 * cos(theta1_sol2 + theta2_sol2);
y_eff2 = y1_s2 + l2 * sin(theta1_sol2 + theta2_sol2);

%% 5. VISUALISASI GRAFIS
figure('Name', 'Inverse Kinematics Robot RR', 'Color', [1 1 1]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)');
title(sprintf('Inverse Kinematics (Target: X = %.2f m, Y = %.2f m)', x_target, y_target));

% Batas Workspace
r_max = l1 + l2;
r_min = abs(l1 - l2);
th_grid = linspace(0, 2*pi, 200);
plot(r_max*cos(th_grid), r_max*sin(th_grid), 'r--', 'LineWidth', 1, 'DisplayName', 'Batas Luar Workspace');
plot(r_min*cos(th_grid), r_min*sin(th_grid), 'b--', 'LineWidth', 1, 'DisplayName', 'Batas Dalam Workspace');

% Plot Titik Target
plot(x_target, y_target, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Target');

% Plot Solusi 1 (Elbow-Down)
plot([0, x1_s1, x_eff1], [0, y1_s1, y_eff1], '-o', 'LineWidth', 2.5, ...
    'MarkerSize', 6, 'Color', [0.2 0.2 0.8], 'MarkerFaceColor', 'b', 'DisplayName', 'Solusi 1 (Elbow-Down)');

% Plot Solusi 2 (Elbow-Up)
plot([0, x1_s2, x_eff2], [0, y1_s2, y_eff2], '--s', 'LineWidth', 2.5, ...
    'MarkerSize', 6, 'Color', [0.8 0.3 0.0], 'MarkerFaceColor', 'r', 'DisplayName', 'Solusi 2 (Elbow-Up)');

xlim([-(r_max+0.2), (r_max+0.2)]);
ylim([-(r_max+0.2), (r_max+0.2)]);
legend('Location', 'northeastoutside');
hold off;