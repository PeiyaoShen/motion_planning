clear all
close all

% 问题配置
way_points = [1 1;
              2 2;
              6 2;
              8 4];
weight = [1;
          1;
          1];
t_arr = [0;
         10;
         20;
         30];
poly_order = 5;

% 解算单维问题
x_way_points = way_points(:, 1);
y_way_points = way_points(:, 2);
[x_param, x_flag] = one_dimen_minimum_snap(t_arr, x_way_points, poly_order, weight);
[y_param, y_flag] = one_dimen_minimum_snap(t_arr, y_way_points, poly_order, weight);

% 显示结果
plot_minimum_snap_curve(x_param, y_param, t_arr, poly_order);



