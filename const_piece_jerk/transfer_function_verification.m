%% 速度规划问题定义
k = 20; %规划离散时间域/优化维数   
dis0 = 1; %初始位置
vel0 = 1; %初始速度
acc0 = 0; %初始加速度
dt = rand(k, 1);    %离散时间间隔序列，此处用随机数验证
jerk = rand(k, 1);  %离散加加速度，此处用随机数验证

%% 初始量转换为向量和矩阵
S0 = ones(k, 1) * dis0;
V0 = ones(k, 1) * vel0;
A0 = ones(k, 1) * acc0;
JERK = jerk;
A = tril(ones(k));
B = tril(ones(k), -1);
T = diag(dt);

%% 矩阵转移公式的k步结果
ACC_FROM_MAT = A0 + A*T*JERK;
VEL_FROM_MAT = V0 + A*T*A0 + (A*T*B*T + 1/2*A*T*T)*JERK;
DIS_FROM_MAT = S0 + A*T*V0 + (A*T*B*T + 1/2*A*T*T)*A0 + (A*T*B*T*B*T+1/2*A*T*B*T*T+1/2*A*T*T*B*T+1/6*A*T*T*T)*JERK;
ACC_FROM_MAT = [acc0; ACC_FROM_MAT];
VEL_FROM_MAT = [vel0; VEL_FROM_MAT];
DIS_FROM_MAT = [dis0; DIS_FROM_MAT];

%% 标量转移公式的k步结果
ACC_FROM_SCALAR = [acc0];
VEL_FROM_SCALAR = [vel0];
DIS_FROM_SCALAR = [dis0];
for i = 1 : 1 : k
    acc = ACC_FROM_SCALAR(i) + jerk(i)*dt(i);
    vel = VEL_FROM_SCALAR(i) + ACC_FROM_SCALAR(i)*dt(i) + 1/2*jerk(i)*dt(i)^2;
    dis = DIS_FROM_SCALAR(i) + VEL_FROM_SCALAR(i)*dt(i) + 1/2*ACC_FROM_SCALAR(i)*dt(i)^2 + 1/6*jerk(i)*dt(i)^3;
    ACC_FROM_SCALAR = [ACC_FROM_SCALAR; acc];
    VEL_FROM_SCALAR = [VEL_FROM_SCALAR; vel];
    DIS_FROM_SCALAR = [DIS_FROM_SCALAR; dis];
end

%% 对比矩阵转移和标量转移
total_time = sum(dt);
origin_time = [0; cumsum(dt)];
figure;
subplot(3,1,1);
hold on;
plot(origin_time, ACC_FROM_MAT, 'b*-', origin_time, ACC_FROM_SCALAR, 'ro-');
xlabel('t [s]');
ylabel('acc [m/s^2]');
legend('mat transfer result', 'scalar transfer result');
hold off;
subplot(3,1,2);
hold on;
plot(origin_time, VEL_FROM_MAT, 'b*-', origin_time, VEL_FROM_SCALAR, 'ro-');
xlabel('t [s]');
ylabel('vel [m/s]');
legend('mat transfer result', 'scalar transfer result');
hold off;
subplot(3,1,3);
hold on;
plot(origin_time, DIS_FROM_MAT, 'b*-', origin_time, DIS_FROM_SCALAR, 'ro-');
xlabel('t [s]');
ylabel('dis [m]');
legend('mat transfer result', 'scalar transfer result');
hold off;
