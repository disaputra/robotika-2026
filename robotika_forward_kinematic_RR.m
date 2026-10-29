%% PROGRAM 1: FORWARD KINEMATICS ROBOT PLANAR 2-DOF (TIPE RR)
% Deskripsi: Menghitung posisi end-effector berdasarkan sudut sendi (theta1, theta2)

clear; clc; close all;

%% 1. PARAMETER ROBOT DAN INPUT SUDUT
l1 = 1.0; % Panjang Link 1 (meter)
l2 = 0.8; % Panjang Link 2 (meter)

% Input Sudut Sendi (dalam derajat)
theta1_deg = 30; % Sudut Sendi 1
theta2_deg = 45; % Sudut Sendi 2

% Konversi ke Radian
theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);

%% 2. PERHITUNGAN FORWARD KINEMATICS
% Posisi Sendi 1 (Ujung Link 1)
x1 = l1 * cos(theta1);
y1 = l1 * sin(theta1);

% Posisi End-Effector (Ujung Link 2)
x_eff = x1 + l2 * cos(theta1 + theta2);
y_eff = y1 + l2 * sin(theta1 + theta2);

%% 3. TAMPILKAN HASIL PADA KONSOL
fprintf('====================================================\n');
fprintf('       SIMULASI FORWARD KINEMATICS ROBOT RR         \n');
fprintf('====================================================\n');
fprintf('Parameter Robot : l1 = %.2f m, l2 = %.2f m\n', l1, l2);
fprintf('Input Sudut     : theta1 = %.2f deg, theta2 = %.2f deg\n', theta1_deg, theta2_deg);
fprintf('----------------------------------------------------\n');
fprintf('Posisi Sendi 1   : X1 = %6.4f m, Y1 = %6.4f m\n', x1, y1);
fprintf('Posisi End-Eff   : X  = %6.4f m, Y  = %6.4f m\n', x_eff, y_eff);
fprintf('====================================================\n\n');

%% 4. VISUALISASI GRAFIS
figure('Name', 'Forward Kinematics Robot RR', 'Color', [1 1 1]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)');
title(sprintf('Forward Kinematics (\\theta_1 = %.1f^o, \\theta_2 = %.1f^o)', theta1_deg, theta2_deg));

% Batas Workspace
r_max = l1 + l2;
r_min = abs(l1 - l2);
th_grid = linspace(0, 2*pi, 200);
plot(r_max*cos(th_grid), r_max*sin(th_grid), 'r--', 'LineWidth', 1, 'DisplayName', 'Batas Luar Workspace');
plot(r_min*cos(th_grid), r_min*sin(th_grid), 'b--', 'LineWidth', 1, 'DisplayName', 'Batas Dalam Workspace');

% Plot Struktur Robot
plot([0, x1, x_eff], [0, y1, y_eff], '-o', 'LineWidth', 3, ...
    'MarkerSize', 8, 'MarkerFaceColor', 'r', 'Color', [0.2 0.2 0.8], 'DisplayName', 'Lengan Robot');

% Plot Titik End-Effector
plot(x_eff, y_eff, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'End-Effector');

xlim([-(r_max+0.2), (r_max+0.2)]);
ylim([-(r_max+0.2), (r_max+0.2)]);
legend('Location', 'northeastoutside');
hold off;