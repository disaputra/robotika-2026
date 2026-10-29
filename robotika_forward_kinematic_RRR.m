%% SIMULASI FORWARD KINEMATICS ROBOT PLANAR 3-DOF (TIPE RRR)
% Mata Kuliah: Robotika
% Catatan: Dijalankan langsung dalam satu script tunggal tanpa fungsi terpisah.

clear; clc; close all;

%% 1. PARAMETER ROBOT
l1 = 1.0; % Panjang Link 1 (meter)
l2 = 0.8; % Panjang Link 2 (meter)
l3 = 0.5; % Panjang Link 3 (meter)

% Input Sudut Target Sendi (dalam derajat)
theta1_deg = 30; 
theta2_deg = 45; 
theta3_deg = -30;

% Konversi Sudut Target ke Radian
th1_target = deg2rad(theta1_deg);
th2_target = deg2rad(theta2_deg);
th3_target = deg2rad(theta3_deg);

%% 2. PERHITUNGAN FORWARD KINEMATICS POSISI AKHIR
x1_final = l1 * cos(th1_target);
y1_final = l1 * sin(th1_target);

x2_final = x1_final + l2 * cos(th1_target + th2_target);
y2_final = y1_final + l2 * sin(th1_target + th2_target);

x3_final = x2_final + l3 * cos(th1_target + th2_target + th3_target);
y3_final = y2_final + l3 * sin(th1_target + th2_target + th3_target);

phi_final_rad = th1_target + th2_target + th3_target;
phi_final_deg = rad2deg(phi_final_rad);

%% 3. TAMPILKAN HASIL PADA KONSOL
fprintf('====================================================\n');
fprintf('     SIMULASI FORWARD KINEMATICS ROBOT RRR (3-DOF)  \n');
fprintf('====================================================\n');
fprintf('Parameter Robot : l1 = %.2f m, l2 = %.2f m, l3 = %.2f m\n', l1, l2, l3);
fprintf('Input Sudut     : theta1 = %.1f deg, theta2 = %.1f deg, theta3 = %.1f deg\n', ...
    theta1_deg, theta2_deg, theta3_deg);
fprintf('----------------------------------------------------\n');
fprintf('Posisi Sendi 1  : X1 = %6.4f m, Y1 = %6.4f m\n', x1_final, y1_final);
fprintf('Posisi Sendi 2  : X2 = %6.4f m, Y2 = %6.4f m\n', x2_final, y2_final);
fprintf('Posisi End-Eff  : X3 = %6.4f m, Y3 = %6.4f m\n', x3_final, y3_final);
fprintf('Orientasi (\phi) : %.2f deg\n', phi_final_deg);
fprintf('====================================================\n\n');

%% 4. INISIALISASI VISUALISASI DAN ANIMASI
figure('Name', 'Forward Kinematics Robot RRR 3-DOF', 'Color', [1 1 1]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)');
title(sprintf('Forward Kinematics RRR (\\theta_1=%.1f^o, \\theta_2=%.1f^o, \\theta_3=%.1f^o)', ...
    theta1_deg, theta2_deg, theta3_deg));

% Batas Maksimum Workspace
r_max = l1 + l2 + l3;
th_grid = linspace(0, 2*pi, 200);
plot(r_max*cos(th_grid), r_max*sin(th_grid), 'r--', 'LineWidth', 1, 'DisplayName', 'Batas Luar Workspace');

% Plot Inisialisasi Lengan Robot (3 Link, 4 Titik Koordinat)
h_arm = plot([0, x1_final], [0, y1_final], '-o', 'LineWidth', 3, ...
    'MarkerSize', 7, 'MarkerFaceColor', 'r', 'Color', [0.2 0.2 0.8], 'DisplayName', 'Lengan Robot RRR');

% Plot End-Effector
h_eff = plot(x3_final, y3_final, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'End-Effector');

xlim([-(r_max+0.2), (r_max+0.2)]);
ylim([-(r_max+0.2), (r_max+0.2)]);
legend('Location', 'northeastoutside');

%% 5. ANIMASI GERAKAN DARI POSISI NOL KE POSISI TARGET
N_steps = 60;
t1_traj = linspace(0, th1_target, N_steps);
t2_traj = linspace(0, th2_target, N_steps);
t3_traj = linspace(0, th3_target, N_steps);

for k = 1:N_steps
    % Perhitungan FK pada step-k
    th1_k = t1_traj(k);
    th2_k = t2_traj(k);
    th3_k = t3_traj(k);
    
    x1_k = l1 * cos(th1_k);
    y1_k = l1 * sin(th1_k);
    
    x2_k = x1_k + l2 * cos(th1_k + th2_k);
    y2_k = y1_k + l2 * sin(th1_k + th2_k);
    
    x3_k = x2_k + l3 * cos(th1_k + th2_k + th3_k);
    y3_k = y2_k + l3 * sin(th1_k + th2_k + th3_k);
    
    % Perbarui Plot Robot
    set(h_arm, 'XData', [0, x1_k, x2_k, x3_k], 'YData', [0, y1_k, y2_k, y3_k]);
    set(h_eff, 'XData', x3_k, 'YData', y3_k);
    
    drawnow;
    pause(0.02);
end

% Gambar Vektor Orientasi End-Effector (Anak Panah Kecil pada Ujung Robot)
arrow_len = 0.3;
quiver(x3_final, y3_final, arrow_len*cos(phi_final_rad), arrow_len*sin(phi_final_rad), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 0.8, 'DisplayName', 'Arah Orientasi (\phi)');

hold off;