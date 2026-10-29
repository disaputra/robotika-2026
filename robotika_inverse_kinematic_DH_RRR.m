%% SIMULASI INVERSE KINEMATICS DENAVIT-HARTENBERG (3-DOF ARTICULATED ROBOT)
% Berdasarkan model DH: Body (Base) -> Shoulder -> Elbow -> End-Effector

clear; clc; close all;

%% 1. PARAMETER GEOMETRI ROBOT (dalam meter)
d1 = 1.0; % Tinggi Base ke Shoulder (m)
a2 = 0.8; % Panjang Link Shoulder ke Elbow (m)
a3 = 0.6; % Panjang Link Elbow ke End-Effector (m)

% Posisi Target End-Effector (X, Y, Z)
X_target = -0.7;
Y_target = 0.2;
Z_target = 2.0;

fprintf('====================================================\n');
fprintf('   SIMULASI DH INVERSE KINEMATICS ROBOT 3-DOF      \n');
fprintf('====================================================\n');
fprintf('Parameter Robot : d1 = %.2f m, a2 = %.2f m, a3 = %.2f m\n', d1, a2, a3);
fprintf('Target Posisi   : X = %.2f m, Y = %.2f m, Z = %.2f m\n', X_target, Y_target, Z_target);
fprintf('----------------------------------------------------\n');

%% 2. PERHITUNGAN INVERSE KINEMATICS
% 1. Hitung Theta 1 (Rotasi Base)
theta1 = atan2(Y_target, X_target);

% 2. Proyeksi Jarak Horisontal (r) dan Vertikal Terhadap Shoulder (s)
r = sqrt(X_target^2 + Y_target^2);
s = Z_target - d1;

% 3. Cek Ketercapaian Target (Workspace Check)
D = (r^2 + s^2 - a2^2 - a3^2) / (2 * a2 * a3);

if abs(D) > 1
    error('Target (%.2f, %.2f, %.2f) berada DI LUAR WORKSPACE robot!', ...
        X_target, Y_target, Z_target);
end

% --- SOLUSI 1: Elbow-Down ---
theta3_sol1 = atan2(sqrt(1 - D^2), D);
theta2_sol1 = atan2(s, r) - atan2(a3 * sin(theta3_sol1), a2 + a3 * cos(theta3_sol1));

% --- SOLUSI 2: Elbow-Up ---
theta3_sol2 = atan2(-sqrt(1 - D^2), D);
theta2_sol2 = atan2(s, r) - atan2(a3 * sin(theta3_sol2), a2 + a3 * cos(theta3_sol2));

fprintf('Solusi 1 (Elbow-Down) : th1 = %6.2f deg, th2 = %6.2f deg, th3 = %6.2f deg\n', ...
    rad2deg(theta1), rad2deg(theta2_sol1), rad2deg(theta3_sol1));
fprintf('Solusi 2 (Elbow-Up)   : th1 = %6.2f deg, th2 = %6.2f deg, th3 = %6.2f deg\n', ...
    rad2deg(theta1), rad2deg(theta2_sol2), rad2deg(theta3_sol2));
fprintf('====================================================\n\n');

%% 3. VERIFIKASI DENGAN FORWARD KINEMATICS (FORWARD TRANSFORMATIONS)
% Fungsi pembentuk matriks transformasi DH
get_A = @(th, d, a, alpha) [
    cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
    sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
    0,        sin(alpha),          cos(alpha),         d;
    0,        0,                   0,                  1
];

% Posisi Titik-Titik Solusi 1
A1_s1 = get_A(theta1, d1, 0, pi/2);
A2_s1 = get_A(theta2_sol1, 0, a2, 0);
A3_s1 = get_A(theta3_sol1, 0, a3, 0);

T01_s1 = A1_s1;
T02_s1 = T01_s1 * A2_s1;
T03_s1 = T02_s1 * A3_s1;

P0 = [0; 0; 0];
P1_s1 = T01_s1(1:3, 4);
P2_s1 = T02_s1(1:3, 4);
P3_s1 = T03_s1(1:3, 4);

% Posisi Titik-Titik Solusi 2
A1_s2 = get_A(theta1, d1, 0, pi/2);
A2_s2 = get_A(theta2_sol2, 0, a2, 0);
A3_s2 = get_A(theta3_sol2, 0, a3, 0);

T01_s2 = A1_s2;
T02_s2 = T01_s2 * A2_s2;
T03_s2 = T02_s2 * A3_s2;

P1_s2 = T01_s2(1:3, 4);
P2_s2 = T02_s2(1:3, 4);
P3_s2 = T03_s2(1:3, 4);

%% 4. VISUALISASI GRAFIS 3D
figure('Name', 'DH Inverse Kinematics 3-DOF Robot', 'Color', [1 1 1]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)'); zlabel('Sumbu Z (m)');
title(sprintf('Inverse Kinematics (Target: X = %.2f m, Y = %.2f m, Z = %.2f m)', ...
    X_target, Y_target, Z_target));

% Plot Titik Target (Bintang Hijau)
plot3(X_target, Y_target, Z_target, 'gp', 'MarkerSize', 15, ...
    'MarkerFaceColor', 'g', 'DisplayName', 'Target');

% Plot Solusi 1: Elbow-Down (Garis Biru)
plot3([P0(1), P1_s1(1), P2_s1(1), P3_s1(1)], ...
      [P0(2), P1_s1(2), P2_s1(2), P3_s1(2)], ...
      [P0(3), P1_s1(3), P2_s1(3), P3_s1(3)], ...
      '-o', 'LineWidth', 3, 'MarkerSize', 7, ...
      'MarkerFaceColor', 'b', 'Color', [0.2 0.2 0.8], 'DisplayName', 'Solusi 1 (Elbow-Down)');

% Plot Solusi 2: Elbow-Up (Garis Putus Merah)
plot3([P0(1), P1_s2(1), P2_s2(1), P3_s2(1)], ...
      [P0(2), P1_s2(2), P2_s2(2), P3_s2(2)], ...
      [P0(3), P1_s2(3), P2_s2(3), P3_s2(3)], ...
      '--s', 'LineWidth', 3, 'MarkerSize', 7, ...
      'MarkerFaceColor', 'r', 'Color', [0.8 0.3 0.0], 'DisplayName', 'Solusi 2 (Elbow-Up)');

% Pengaturan Batas Tampilan
r_total = d1 + a2 + a3;
xlim([-r_total, r_total]);
ylim([-r_total, r_total]);
zlim([0, r_total]);
view(135, 25);
legend('Location', 'northeast');
hold off;