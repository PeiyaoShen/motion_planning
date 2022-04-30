clc
close all

%% 该脚本主要作用：
%  1.验证piece_jerk_qp优化推导的正确性
%  2.验证每段内的位置、速度、加速度是否会"凸包"


%% 速度规划问题定义
k = 5; %规划离散时间域/优化维数   
dis0 = 1; %初始位置
vel0 = 1; %初始速度
acc0 = 0; %初始加速度
dis_ref = zeros(k, 1); %参考位置
vel_ref = zeros(k, 1); %参考速度
acc_ref = zeros(k, 1); %参考加速度
jerk_ref = zeros(k, 1); %参考加加速度
dis_low = ones(k, 1)*-1; %位置下限
dis_upp = ones(k, 1)*2;  %位置上限
vel_low = ones(k, 1)*-2; %速度下限
vel_upp = ones(k, 1)*2;  %速度上限
acc_low = ones(k, 1)*-1; %加速度下限
acc_upp = ones(k, 1)*1;  %加速度上限
jerk_low = ones(k, 1)*-1;%加加速度下限
jerk_upp = ones(k, 1)*1; %加加速度上限
dt = ones(k, 1);    %离散时间间隔序列 
w1 = ones(k, 1);     %位置权重
w2 = ones(k, 1);          %速度权重
w3 = ones(k, 1);          %加速度权重
w4 = ones(k, 1);          %加加速度权重

%% 规划问题转换为优化问题
S0 = ones(k, 1) * dis0;
V0 = ones(k, 1) * vel0;
A0 = ones(k, 1) * acc0;
S_REF = dis_ref;
V_REF = vel_ref;
A_REF = acc_ref;
J_REF = jerk_ref;
S_LOW = dis_low;
S_UPP = dis_upp;
V_LOW = vel_low;
V_UPP = vel_upp;
A_LOW = acc_low;
A_UPP = acc_upp;
J_LOW = jerk_low;
J_UPP = jerk_upp;
W1 = diag(w1);
W2 = diag(w2);
W3 = diag(w3);
W4 = diag(w4);

A = tril(ones(k));
B = tril(ones(k), -1);
T = diag(dt);
C = A*T*B*T*B*T + 1/2*A*T*B*T*T + 1/2*A*T*T*B*T + 1/6*A*T*T*T;
D = S0 + A*T*V0 + (A*T*B*T + 1/2*A*T*T)*A0 - S_REF;
E = A*T*B*T + 1/2*A*T*T;
F = V0 + A*T*A0 - V_REF;
M = A*T;
N = A0 - A_REF;
P = eye(k);
Q = zeros(k,1) - J_REF;
H = 2 * (C'*W1*C + E'*W2*E + M'*W3*M + P'*W4*P);
U = 2 * (D'*W1*C + F'*W2*E + N'*W3*M + Q'*W4*P)';
G = [-C; 
      C;
     -E; 
      E; 
     -M; 
      M; 
     -P; 
      P];
L = [-S_LOW + D + S_REF; 
      S_UPP - D - S_REF; 
     -V_LOW + F + V_REF; 
      V_UPP - F - V_REF;
     -A_LOW + N + A_REF; 
      A_UPP - N - A_REF;
     -J_LOW + Q + J_REF; 
      J_UPP - Q - J_REF];

%% 求解优化问题
[x,fval,exitflag,output,lambda] = quadprog(H, U, G, L);

%% 优化结果转换为规划结果
opt_jerk = [x; x(end)];
total_time = sum(dt);
origin_time = [0; cumsum(dt)];
origin_num = length(origin_time);
sample_num = 5*k; 
sample_time = linspace(0, total_time, sample_num); % 在规划时间上做更细致的离散

opt_dis = [dis0];
opt_vel = [vel0];
opt_acc = [acc0];
for i = 2: 1 : origin_num
    te = origin_time(i);
    ts = origin_time(i-1);
    tt = te - ts;
    last_jerk = opt_jerk(i-1);
    last_acc = opt_acc(i-1);
    last_vel = opt_vel(i-1);
    last_dis = opt_dis(i-1);
    acc = last_acc + last_jerk * tt;
    vel = last_vel + last_acc*tt + 1/2*last_jerk*tt^2;
    dis = last_dis + last_vel*tt + 1/2*last_acc*tt^2 + 1/6*last_jerk*tt^3;
    opt_dis = [opt_dis; dis];
    opt_vel = [opt_vel; vel];
    opt_acc = [opt_acc; acc];
end

sample_dis = [];
sample_vel = [];
sample_acc = [];
for i = 1 : 1 : sample_num
    te = sample_time(i);
    idx_arr = find(origin_time <= te);
    idx = idx_arr(end);
    ts = origin_time(idx);
    tt = te - ts;
    last_jerk = opt_jerk(idx);
    last_acc = opt_acc(idx);
    last_vel = opt_vel(idx);
    last_dis = opt_dis(idx);
    acc = last_acc + last_jerk * tt;
    vel = last_vel + last_acc*tt + 1/2*last_jerk*tt^2;
    dis = last_dis + last_vel*tt + 1/2*last_acc*tt^2 + 1/6*last_jerk*tt^3;
    sample_dis = [sample_dis; dis];
    sample_vel = [sample_vel; vel];
    sample_acc = [sample_acc; acc];
end

%% 绘图
figure(1)
subplot(4, 1, 1)
hold on;
xlabel('t [s]');
ylabel('dis [m]');
plot(origin_time, opt_dis, 'bo-', sample_time, sample_dis, 'r*--', origin_time(2:end), dis_ref, 'g+-');
plot(origin_time(2:end), dis_low, origin_time(2:end), dis_upp);
legend('opt', 'sample', 'ref', 'low', 'upp');
hold off;
grid on;
subplot(4, 1, 2)
xlabel('t [s]');
ylabel('vel [m/s]');
hold on;
plot(origin_time, opt_vel, 'bo-', sample_time, sample_vel, 'r*--', origin_time(2:end), vel_ref, 'g+-');
plot(origin_time(2:end), vel_low, origin_time(2:end), vel_upp);
legend('opt', 'sample', 'ref', 'low', 'upp');
hold off;
grid on;
subplot(4, 1, 3)
xlabel('t [s]');
ylabel('acc [m/s^2]');
hold on;
plot(origin_time, opt_acc, 'bo-', sample_time, sample_acc, 'r*--', origin_time(2:end), acc_ref, 'g+-');
plot(origin_time(2:end), acc_low, origin_time(2:end), acc_upp);
legend('opt', 'sample', 'ref', 'low', 'upp');
hold off;
grid on;
subplot(4, 1, 4)
xlabel('t [s]');
ylabel('jerk [m/s^3]');
hold on;
plot(origin_time, opt_jerk, 'bo-', origin_time(2:end), jerk_ref, 'g+-');
plot(origin_time(2:end), jerk_low, origin_time(2:end), jerk_upp);
hold off;
legend('opt', 'ref', 'low', 'upp');
grid on;
