%% SIMULASI DINAMIKA ROBOT ARTICULATED 3-DOF (TIPE RRR - DH MODEL)
% Topik: Forward Dynamics & Pengendalian Torsi Sendi 3D
% Catatan: Dijalankan langsung dalam satu script tunggal tanpa fungsi terpisah.

clear; clc; close all;

%% 1. PARAMETER FISIK DAN GEOMETRI ROBOT
m1 = 3.0;    % Massa Link 1 / Base Assembly (kg)
m2 = 2.5;    % Massa Link 2 / Upper Arm (kg)
m3 = 1.5;    % Massa Link 3 / Forearm (kg)

d1 = 1.0;    % Tinggi Base ke Shoulder (m)
a2 = 0.8;    % Panjang Link Shoulder ke Elbow (m)
a3 = 0.6;    % Panjang Link Elbow ke End-Effector (m)

lc1 = d1 / 2; % Pusat massa Link 1 dari Base (m)
lc2 = a2 / 2; % Pusat massa Link 2 dari Shoulder (m)
lc3 = a3 / 2; % Pusat massa Link 3 dari Elbow (m)

I1 = 0.25;   % Momen inersia Link 1 (kg*m^2)
I2 = 0.20;   % Momen inersia Link 2 (kg*m^2)
I3 = 0.10;   % Momen inersia Link 3 (kg*m^2)

g = 9.81;    % Percepatan gravitasi (m/s^2)

%% 2. TARGET POSISI & GAIN KENDALI PD
% Target Sudut Sendi [theta1; theta2; theta3] (dalam radian)
q_target = [deg2rad(45); deg2rad(30); deg2rad(-20)];

% Matriks Gain Kendali PD (Proportional-Derivative) 3x3
Kp = diag([250, 300, 150]);
Kd = diag([40,  50,  20]);

%% 3. PARAMETER SIMULASI NUMERIK
dt = 0.002;          % Langkah waktu integrasi / sampling time (s)
t_end = 4.0;         % Total durasi simulasi (s)
t = 0:dt:t_end;
N = length(t);

% Pre-alokasi matriks data (3 x N)
q   = zeros(3, N);   % Posisi sudut sendi [theta1; theta2; theta3]
dq  = zeros(3, N);   % Kecepatan sudut sendi [dtheta1; dtheta2; dtheta3]
ddq = zeros(3, N);   % Percepatan sudut sendi [ddtheta1; ddtheta2; ddtheta3]
tau = zeros(3, N);   % Torsi aktuator sendi [tau1; tau2; tau3]

% Kondisi Awal Robot (Awalnya diam pada posisi 0 radian)
q(:, 1)  = [deg2rad(0); deg2rad(0); deg2rad(0)];
dq(:, 1) = [0; 0; 0];

%% 4. LOOP INTEGRASI NUMERIK (FORWARD DYNAMICS)
for k = 1:N-1
    % Ambil variabel status saat ini
    q1  = q(1, k);  q2  = q(2, k);  q3  = q(3, k);
    dq1 = dq(1, k); dq2 = dq(2, k); dq3 = dq(3, k);
    
    % --- Formulasi Matriks Inersia M(q) 3x3 ---
    % Sifat: Link 1 terpisah secara rotasi azimuthal, Link 2 & 3 saling terkoppel
    c2 = cos(q2);   s2 = sin(q2);
    c3 = cos(q3);   s3 = sin(q3);
    c23 = cos(q2 + q3); s23 = sin(q2 + q3);
    
    % Inersia Rotasi Base terhadap Z0
    r1_eff = a2*c2 + lc3*c23;
    r2_eff = lc2*c2;
    M11 = I1 + m2*r2_eff^2 + m3*r1_eff^2;
    
    % Inersia Planar Shoulder-Elbow (Bidang Pitch)
    M22 = I2 + I3 + m2*lc2^2 + m3*(a2^2 + lc3^2 + 2*a2*lc3*c3);
    M23 = I3 + m3*(lc3^2 + a2*lc3*c3);
    M32 = M23;
    M33 = I3 + m3*lc3^2;
    
    M = [M11, 0,   0;
         0,   M22, M23;
         0,   M32, M33];
    
    % --- Formulasi Matriks Coriolis & Sentrifugal C(q, dq) 3x3 ---
    h_c3 = m3*a2*lc3*s3;
    
    C11 = - (m2*lc2^2*s2*c2 + m3*r1_eff*(a2*s2 + lc3*s23))*dq2 - m3*r1_eff*lc3*s23*dq3;
    C12 = C11;
    C13 = 0;
    
    C21 = (m2*lc2^2*s2*c2 + m3*r1_eff*(a2*s2 + lc3*s23))*dq1;
    C22 = -h_c3 * dq3;
    C23 = -h_c3 * (dq2 + dq3);
    
    C31 = m3*r1_eff*lc3*s23*dq1;
    C32 = h_c3 * dq2;
    C33 = 0;
    
    C = [C11, C12, C13;
         C21, C22, C23;
         C31, C32, C33];
      
    % --- Formulasi Vektor Gravitasi G(q) 3x1 ---
    % Sendi 1 (Base) berputar horizontal -> tidak terpengaruh gravitasi
    G1 = 0;
    G2 = (m2*lc2 + m3*a2)*g*c2 + m3*lc3*g*c23;
    G3 = m3*lc3*g*c23;
    G = [G1; G2; G3];
    
    % --- Perhitungan Torsi Kendali PD + Kompensasi Gravitasi ---
    e  = q_target - q(:, k);   % Eror posisi
    de = -dq(:, k);            % Eror kecepatan (target dq = 0)
    
    tau(:, k) = Kp*e + Kd*de + G; 
    
    % --- Perhitungan Percepatan Sendi (Forward Dynamics): ddq = M \ (tau - C*dq - G) ---
    ddq(:, k) = M \ (tau(:, k) - C*dq(:, k) - G);
    
    % --- Integrasi Numerik Euler ---
    dq(:, k+1) = dq(:, k) + ddq(:, k) * dt;
    q(:, k+1)  = q(:, k)  + dq(:, k+1) * dt;
end

% Mengisi nilai torsi pada langkah waktu terakhir
tau(:, N) = tau(:, N-1);

%% 5. VISUALISASI HASIL SIMULASI
figure('Name', 'Simulasi Dinamika & Kendali Robot 3-DOF RRR', 'Color', [1 1 1]);

% Subplot 1: Respon Posisi Sudut Sendi
subplot(2, 2, 1);
plot(t, rad2deg(q(1, :)), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, rad2deg(q(2, :)), 'g', 'LineWidth', 1.5);
plot(t, rad2deg(q(3, :)), 'b', 'LineWidth', 1.5);
% yline(rad2deg(q_target(1)), 'r--', '\theta_1 target');
% yline(rad2deg(q_target(2)), 'g--', '\theta_2 target');
% yline(rad2deg(q_target(3)), 'b--', '\theta_3 target');
xlabel('Waktu (s)'); ylabel('Sudut (deg)');
title('Respon Posisi Sudut Sendi');
legend('\theta_1 (Base)', '\theta_2 (Shoulder)', '\theta_3 (Elbow)', 'Location', 'southeast');

% Subplot 2: Profil Torsi Aktuator Sendi
subplot(2, 2, 2);
plot(t, tau(1, :), 'r', 'LineWidth', 1.5); hold on; grid on;
plot(t, tau(2, :), 'g', 'LineWidth', 1.5);
plot(t, tau(3, :), 'b', 'LineWidth', 1.5);
xlabel('Waktu (s)'); ylabel('Torsi (Nm)');
title('Profil Torsi Penggerak Sendi (\tau)');
legend('\tau_1', '\tau_2', '\tau_3', 'Location', 'northeast');

% Subplot 3 & 4: Animasi Pergerakan 3D Lengan Robot
subplot(2, 2, [3, 4]);
hold on; grid on; axis equal;
xlabel('Sumbu X (m)'); ylabel('Sumbu Y (m)'); zlabel('Sumbu Z (m)');
title('Animasi Gerak Dinamis 3D Lengan Robot 3-DOF');

r_total = d1 + a2 + a3;
xlim([-r_total, r_total]);
ylim([-r_total, r_total]);
zlim([0, r_total]);
view(135, 25);

% Plot elemen robot awal
h_arm = plot3([0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], '-o', 'LineWidth', 3.5, ...
    'MarkerSize', 7, 'MarkerFaceColor', 'r', 'Color', [0.2 0.2 0.8]);

% Visualisasi titik target menggunakan Forward Kinematics DH
x_t = (a2*cos(q_target(2)) + a3*cos(q_target(2)+q_target(3))) * cos(q_target(1));
y_t = (a2*cos(q_target(2)) + a3*cos(q_target(2)+q_target(3))) * sin(q_target(1));
z_t = d1 + a2*sin(q_target(2)) + a3*sin(q_target(2)+q_target(3));
plot3(x_t, y_t, z_t, 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Target');

% Loop pembaruan animasi
step_anim = round(0.04 / dt); % Tampilkan bingkai setiap 0.04 detik
for k = 1:step_anim:N
    % Forward Kinematics pada step k
    th1_k = q(1, k); th2_k = q(2, k); th3_k = q(3, k);
    
    P0 = [0; 0; 0];
    P1 = [0; 0; d1];
    P2 = [ (a2*cos(th2_k)) * cos(th1_k);
           (a2*cos(th2_k)) * sin(th1_k);
           d1 + a2*sin(th2_k) ];
    P3 = [ (a2*cos(th2_k) + a3*cos(th2_k+th3_k)) * cos(th1_k);
           (a2*cos(th2_k) + a3*cos(th2_k+th3_k)) * sin(th1_k);
           d1 + a2*sin(th2_k) + a3*sin(th2_k+th3_k) ];
    
    set(h_arm, 'XData', [P0(1), P1(1), P2(1), P3(1)], ...
               'YData', [P0(2), P1(2), P2(2), P3(2)], ...
               'ZData', [P0(3), P1(3), P2(3), P3(3)]);
    drawnow;
end