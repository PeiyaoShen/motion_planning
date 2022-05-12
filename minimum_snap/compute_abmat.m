function [aeq, beq] = compute_abmat(t_arr, p_arr, celld2s)
%compute_abmat 计算ab等式系数

dis = p_arr(:, 1);
vel = p_arr(:, 2);
acc = p_arr(:, 3);
jer = p_arr(:, 4);
snp = p_arr(:, 5);
[r, c] = size(t_arr);
k = r - 1;

dist_poly = celld2s(1);
dist_poly = cell2mat(dist_poly);
dist_fac = dist_poly(:,1);
dist_pow = dist_poly(:,2);
velo_poly = celld2s(2);
velo_poly = cell2mat(velo_poly);
velo_fac = velo_poly(:,1);
velo_pow = velo_poly(:,2);
acce_poly = celld2s(3);
acce_poly = cell2mat(acce_poly);
acce_fac = acce_poly(:,1);
acce_pow = acce_poly(:,2);
jerk_poly = celld2s(4);
jerk_poly = cell2mat(jerk_poly);
jerk_fac = jerk_poly(:,1);
jerk_pow = jerk_poly(:,2);
snap_poly = celld2s(5);
snap_poly = cell2mat(snap_poly);
snap_fac = snap_poly(:,1);
snap_pow = snap_poly(:,2);
n = cell2mat(celld2s(6));

distmat = zeros(1+k, k*(n+1));
distvec = zeros(1+k, 1);
distmat(1, 1:n+1) = (dist_fac.*(t_arr(1).^dist_pow))';
for i = 1 : 1 : k
    distmat(i+1, 1+(i-1)*(n+1) : n+1+(i-1)*(n+1)) = (dist_fac.*(t_arr(i+1).^dist_pow))';
end
distvec = dis;

vamat = zeros(2*2, k*(n+1));
vavec = zeros(2*2, 1);
vamat(1, 1:n+1) = (velo_fac.*(t_arr(1).^velo_pow))';
vamat(2, 1:n+1) = (acce_fac.*(t_arr(1).^acce_pow))';
vamat(3, 1+(k-1)*(n+1):n+1+(k-1)*(n+1)) = (velo_fac.*(t_arr(1+k).^velo_pow))';
vamat(4, 1+(k-1)*(n+1):n+1+(k-1)*(n+1)) = (acce_fac.*(t_arr(1+k).^acce_pow))';
vavec(1) = vel(1);
vavec(2) = acc(1);
vavec(3) = vel(1+k);
vavec(4) = acc(1+k);

d_conmat = zeros(k-1, k*(n+1));
d_convec = zeros(k-1, 1);
for i = 1 : 1 : k-1
    d_conmat(i, 1+(i-1)*(n+1) : n+1+(i-1)*(n+1)) = (dist_fac.*(t_arr(i+1).^dist_pow))';
    d_conmat(i, 1+(i)*(n+1) : n+1+(i)*(n+1)) = -(dist_fac.*(t_arr(i+1).^dist_pow))';
end

v_conmat = zeros(k-1, k*(n+1));
v_convec = zeros(k-1, 1);
for i = 1 : 1 : k-1
    v_conmat(i, 1+(i-1)*(n+1) : n+1+(i-1)*(n+1)) = (velo_fac.*(t_arr(i+1).^velo_pow))';
    v_conmat(i, 1+(i)*(n+1) : n+1+(i)*(n+1)) = -(velo_fac.*(t_arr(i+1).^velo_pow))';
end

a_conmat = zeros(k-1, k*(n+1));
a_convec = zeros(k-1, 1);
for i = 1 : 1 : k-1
    a_conmat(i, 1+(i-1)*(n+1) : n+1+(i-1)*(n+1)) = (acce_fac.*(t_arr(i+1).^acce_pow))';
    a_conmat(i, 1+(i)*(n+1) : n+1+(i)*(n+1)) = -(acce_fac.*(t_arr(i+1).^acce_pow))';
end

aeq = [distmat; vamat; d_conmat; v_conmat; a_conmat];
beq = [distvec; vavec; d_convec; v_convec; a_convec];

end