%% SIMULASI FORWARD KINEMATICS DENAVIT-HARTENBERG (3-DOF ARTICULATED ROBOT)
% Berdasarkan model: Body (Base) -> Shoulder -> Elbow -> End-Effector

clear; clc; close all;

%% 1. PARAMETER GEOMETRI ROBOT (dalam meter)
d1 = 1.0; % Tinggi Base ke Shoulder (m)
a2 = 0.8; % Panjang Link Shoulder ke Elbow (m)
a3 = 0.6; % Panjang Link Elbow ke End-Effector (m)

% Input Sudut Sendi (dalam derajat)
theta1_deg = 0; % Rotasi Base (theta1)
theta2_deg = 30; % Rotasi Shoulder (theta2)
theta3_deg = -20; % Rotasi Elbow (theta3)

% Konversi ke Radian
th1 = deg2rad(theta1_deg);
th2 = deg2rad(theta2_deg);
th3 = deg2rad(theta3_deg);

%% 2. TABEL PARAMETER DENAVIT-HARTENBERG (DH)
% Format Matriks DH: [theta, d, a, alpha]
DH = [
    th1, d1, 0,  pi/2;  % Link 1 (Base - Shoulder)
    th2, 0,  a2, 0;     % Link 2 (Shoulder - Elbow)
    th3, 0,  a3, 0      % Link 3 (Elbow - End-Effector)
];

%% 3. PERHITUNGAN MATRIKS TRANSFORMASI DH
% Fungsi Homogeneous Transformation Matrix DH: A_i
% A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
A = cell(1, 3);
for i = 1:3
    theta = DH(i,1);
    d     = DH(i,2);
    a     = DH(i,3);
    alpha = DH(i,4);
    
    A{i} = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,           sin(alpha),             cos(alpha),            d;
            0,           0,                      0,                     1];
end

% Transformasi Kumulatif
T0_1 = A{1};            % Kerangka 1 terhadap Base
T0_2 = T0_1 * A{2};     % Kerangka 2 terhadap Base
T0_3 = T0_2 * A{3};     % Kerangka 3 (End-Effector) terhadap Base

%% 4. EKSTRAKSI POSISI SENDI & END-EFFECTOR
P0 = [0; 0; 0];         % Titik Base (Origin)
P1 = T0_1(1:3, 4);      % Posisi Sendi Shoulder (Frame 1)
P2 = T0_2(1:3, 4);      % Posisi Sendi Elbow (Frame 2)
P3 = T0_3(1:3, 4);      % Posisi End-Effector (Frame 3)

%% 5. TAMPILKAN HASIL PADA KONSOL
fprintf('====================================================\n');
fprintf('   SIMULASI DH FORWARD KINEMATICS ROBOT 3-DOF      \n');
fprintf('====================================================\n');
fprintf('Input Sudut : theta1 = %.1f deg, theta2 = %.1f deg, theta3 = %.1f deg\n', ...
    theta1_deg, theta2_deg, theta3_deg);
fprintf('----------------------------------------------------\n');
fprintf('Posisi Shoulder (P1) : X = %6.4f m, Y = %6.4f m, Z = %6.4f m\n', P1(1), P1(2), P1(3));
fprintf('Posisi Elbow    (P2) : X = %6.4f m, Y = %6.4f m, Z = %6.4f m\n', P2(1), P2(2), P2(3));
fprintf('Posisi End-Eff  (P3) : X = %6.4f m, Y = %6.4f m, Z = %6.4f m\n', P3(1), P3(2), P3(3));
fprintf('====================================================\n\n');

%% 6. VISUALISASI GRAFIS 3D
figure('Name', 'DH Forward Kinematics 3-DOF Robot', 'Color', [1 1 1]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)'); zlabel('Sumbu Z (m)');
title(sprintf('Model Robot 3-DOF (\\theta_1=%.1f^o, \\theta_2=%.1f^o, \\theta_3=%.1f^o)', ...
    theta1_deg, theta2_deg, theta3_deg));

% Plot Struktur Lengan Robot
plot3([P0(1), P1(1), P2(1), P3(1)], ...
      [P0(2), P1(2), P2(2), P3(2)], ...
      [P0(3), P1(3), P2(3), P3(3)], ...
      '-o', 'LineWidth', 4, 'MarkerSize', 8, ...
      'MarkerFaceColor', 'r', 'Color', [0.2 0.2 0.8], 'DisplayName', 'Lengan Robot');

% Plot End-Effector
plot3(P3(1), P3(2), P3(3), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'End-Effector');

% Plot Sumbu Koordinat (Frame Base & Frame End-Effector)
scale = 0.3;
quiver3(P0(1), P0(2), P0(3), scale, 0, 0, 'r', 'LineWidth', 2, 'HandleVisibility', 'off');
quiver3(P0(1), P0(2), P0(3), 0, scale, 0, 'g', 'LineWidth', 2, 'HandleVisibility', 'off');
quiver3(P0(1), P0(2), P0(3), 0, 0, scale, 'b', 'LineWidth', 2, 'HandleVisibility', 'off');

% Pengaturan Batas Tampilan
r_total = d1 + a2 + a3;
xlim([-r_total, r_total]);
ylim([-r_total, r_total]);
zlim([0, r_total]);
view(135, 25); % Sudut pandang 3D
legend('Location', 'northeast');
hold off;