function [param, exit_flag] = one_dimen_minimum_snap(t_arr, one_demin_waypoints, poly_order, weight)
%one_dimen_minimum_snap 计算最小snap的一维曲线
[r, c] = size(one_demin_waypoints);
k = r - 1;
n = poly_order;
p_arr = [one_demin_waypoints zeros(k + 1, 4)];

celld2s = curve_pow(n);
q = compute_qmat(t_arr, celld2s, weight);
[aeq, beq] = compute_abmat(t_arr, p_arr, celld2s);
h = 2 * q;
f = zeros((n+1)*k, 1);

[x, fval, exit_flag, output, lambda] = quadprog(h, f, [], [], aeq, beq);
param = x;
end