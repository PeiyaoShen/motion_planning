function [celld2s] = curve_pow(high_order)
%curve_pow 计算high_order阶多项式曲线关于自变量t的幂次项

distance_tpow = [];
distance_tfac = [];
velocity_tpow = [];
velocity_tfac = [];
acceleration_tpow = [];
acceleration_tfac = [];
jerk_tpow = [];
jerk_tfac = [];
snap_tpow = [];
snap_tfac = [];

for i = 0 : 1 : high_order
    power = i;
    distance_tpow  = [distance_tpow; power];
    velocity_tpow  = [velocity_tpow; power - 1];
    acceleration_tpow  = [acceleration_tpow; power - 2];
    jerk_tpow  = [jerk_tpow; power - 3];
    snap_tpow  = [snap_tpow; power - 4];
    distance_tfac = [distance_tfac; 1];
    velocity_tfac = [velocity_tfac; power];
    acceleration_tfac = [acceleration_tfac; power*(power-1)];
    jerk_tfac = [jerk_tfac; power*(power-1)*(power-2)];
    snap_tfac = [snap_tfac; power*(power-1)*(power-2)*(power-3)];
end

idx = find(distance_tpow < 0);
distance_tpow(idx) = 0;
idx = find(velocity_tpow < 0);
velocity_tpow(idx) = 0;
idx = find(acceleration_tpow < 0);
acceleration_tpow(idx) = 0;
idx = find(jerk_tpow < 0);
jerk_tpow(idx) = 0;
idx = find(snap_tpow < 0);
snap_tpow(idx) = 0;

celld2s = {[distance_tfac distance_tpow], [velocity_tfac velocity_tpow], [acceleration_tfac acceleration_tpow], [jerk_tfac jerk_tpow], [snap_tfac snap_tpow], high_order};
end