%% SIMULASI DINAMIKA ROBOT PLANAR 2-DOF (TIPE RR)
% Topik: Forward Dynamics & Pengendalian Torsi Sendi
% Catatan: Dijalankan langsung dalam satu script tunggal tanpa fungsi terpisah.

clear; clc; close all;

%% 1. PARAMETER FISIK ROBOT
m1 = 2.0;    % Massa Link 1 (kg)
m2 = 1.5;    % Massa Link 2 (kg)
l1 = 1.0;    % Panjang Link 1 (m)
l2 = 0.8;    % Panjang Link 2 (m)
lc1 = 0.5;   % Pusat massa Link 1 dari sendi 1 (m)
lc2 = 0.4;   % Pusat massa Link 2 dari sendi 2 (m)
I1 = 0.15;   % Momen inersia Link 1 (kg*m^2)
I2 = 0.10;   % Momen inersia Link 2 (kg*m^2)
g = 9.81;    % Percepatan gravitasi (m/s^2)

%% 2. TARGET POSISI & GAIN KENDALI PD
% Target Sudut Sendi [theta1; theta2] (dalam radian)
q_target = [deg2rad(45); deg2rad(60)];

% Matriks Gain Kendali PD (Proportional-Derivative)
Kp = [150, 0; 0, 100];
Kd = [30, 0; 0, 20];

%% 3. PARAMETER SIMULASI NUMERIK
dt = 0.002;             % Langkah waktu integrasi / sampling time (s)
t_end = 4.0;            % Total durasi simulasi (s)
t = 0:dt:t_end;
N = length(t);

% Pre-alokasi matriks data
q   = zeros(2, N);      % Posisi sudut sendi [theta1; theta2]
dq  = zeros(2, N);      % Kecepatan sudut sendi [dtheta1; dtheta2]
ddq = zeros(2, N);      % Percepatan sudut sendi [ddtheta1; ddtheta2]
tau = zeros(2, N);      % Torsi aktuator sendi [tau1; tau2]

% Kondisi Awal Robot (Awalnya diam pada posisi 0 derajat)
q(:, 1)  = [deg2rad(0); deg2rad(0)];
dq(:, 1) = [0; 0];

%% 4. LOOP INTEGRASI NUMERIK (FORWARD DYNAMICS)
for k = 1:N-1
    % Ambil variabel status saat ini
    q1  = q(1, k);
    q2  = q(2, k);
    dq1 = dq(1, k);
    dq2 = dq(2, k);
    
    % --- Formulasi Matriks Inersia M(q) ---
    M11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I2;
    M12 = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
    M21 = M12;
    M22 = m2*lc2^2 + I2;
    M = [M11, M12; M21, M22];
    
    % --- Formulasi Matriks Coriolis & Sentrifugal C(q, dq) ---
    h = m2*l1*lc2*sin(q2);
    C = [-h*dq2, -h*(dq1 + dq2); ...
          h*dq1,          0     ];
      
    % --- Formulasi Vektor Gravitasi G(q) ---
    G1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    G2 = m2*lc2*g*cos(q1 + q2);
    G = [G1; G2];
    
    % --- Perhitungan Torsi Kendali PD + Kompensasi Gravitasi ---
    e  = q_target - q(:, k);   % Eror posisi
    de = -dq(:, k);            % Eror kecepatan (target dq = 0)
    tau(:, k) = Kp*e + Kd*de + G; 
    
    % --- Perhitungan Percepatan Sendi: ddq = M \ (tau - C*dq - G) ---
    ddq(:, k) = M \ (tau(:, k) - C*[dq1; dq2] - G);
    
    % --- Integrasi Numerik Euler ---
    dq(:, k+1) = dq(:, k) + ddq(:, k) * dt;
    q(:, k+1)  = q(:, k)  + dq(:, k+1) * dt;
end

% Mengisi nilai torsi pada langkah waktu terakhir
tau(:, N) = tau(:, N-1);

%% 5. VISUALISASI HASIL SIMULASI
figure('Name', 'Simulasi Dinamika & Kendali Robot RR', 'Color', [1 1 1]);

% Subplot 1: Respon Posisi Sudut Sendi
subplot(2, 2, 1);
plot(t, rad2deg(q(1, :)), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, rad2deg(q(2, :)), 'b', 'LineWidth', 1.5);
% yline(rad2deg(q_target(1)), 'r--', 'Target \theta_1');
% yline(rad2deg(q_target(2)), 'b--', 'Target \theta_2');
xlabel('Waktu (s)'); ylabel('Sudut (deg)');
title('Respon Posisi Sudut Sendi');
legend('\theta_1', '\theta_2', 'Location', 'southeast');

% Subplot 2: Profil Torsi Aktuator Sendi
subplot(2, 2, 2);
plot(t, tau(1, :), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, tau(2, :), 'b', 'LineWidth', 1.5);
xlabel('Waktu (s)'); ylabel('Torsi (Nm)');
title('Profil Torsi Penggerak Sendi (\tau)');
legend('\tau_1', '\tau_2', 'Location', 'northeast');

% Subplot 3 & 4: Animasi Pergerakan Lengan Robot
subplot(2, 2, [3, 4]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)');
title('Animasi Gerak Dinamis Lengan Robot RR');

r_max = l1 + l2;
xlim([-(r_max+0.2), (r_max+0.2)]);
ylim([-(r_max+0.2), (r_max+0.2)]);

% Plot elemen robot
h_arm = plot([0, 0, 0], [0, 0, 0], '-o', 'LineWidth', 3, ...
    'MarkerSize', 8, 'MarkerFaceColor', 'r', 'Color', [0.2 0.2 0.8]);

% Visualisasi titik target
x_t = l1*cos(q_target(1)) + l2*cos(q_target(1) + q_target(2));
y_t = l1*sin(q_target(1)) + l2*sin(q_target(1) + q_target(2));
plot(x_t, y_t, 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

% Loop pembaruan animasi
step_anim = round(0.04 / dt); % Tampilkan bingkai setiap 0.04 detik
for k = 1:step_anim:N
    x1 = l1 * cos(q(1, k));
    y1 = l1 * sin(q(1, k));
    x2 = x1 + l2 * cos(q(1, k) + q(2, k));
    y2 = y1 + l2 * sin(q(1, k) + q(2, k));
    
    set(h_arm, 'XData', [0, x1, x2], 'YData', [0, y1, y2]);
    drawnow;
end