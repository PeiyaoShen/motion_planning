function plot_minimum_snap_curve(x_param,y_param, t_arr, poly_order)
%plot_minimum_snap_curve 制作和显示绘图数据

[r, c] = size(t_arr);
k = r - 1;
n = poly_order;

figure(1)
title('distance of x');
xlabel('t [s]');
ylabel('distance [m]');
hold on;
for i = 1 : 1 : k
    param = x_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    d = polyval(param, t);
    plot(t, d);
end
hold off;
grid on;

figure(2)
title('velocity of x');
xlabel('t [s]');
ylabel('velocity [m/s]');
hold on;
for i = 1 : 1 : k
    param = x_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    v = polyval(param, t);
    plot(t, v);
end
hold off;
grid on;

figure(3)
title('acceleration of x');
xlabel('t [s]');
ylabel('acceleration [m/s^2]');
hold on;
for i = 1 : 1 : k
    param = x_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    param = polyder(param);
    a = polyval(param, t);
    plot(t, a);
end
hold off;
grid on;

figure(4)
title('jerk of x');
xlabel('t [s]');
ylabel('jerk [m/s^3]');
hold on;
for i = 1 : 1 : k
    param = x_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    param = polyder(param);
    param = polyder(param);
    j = polyval(param, t);
    plot(t, j);
end
hold off;
grid on;

figure(5)
title('snap of x');
xlabel('t [s]');
ylabel('snap [m/s^4]');
hold on;
for i = 1 : 1 : k
    param = x_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    param = polyder(param);
    param = polyder(param);
    param = polyder(param);
    s = polyval(param, t);
    plot(t, s);
end
hold off;
grid on;

figure(6)
title('distance of y');
xlabel('t [s]');
ylabel('distance [m]');
hold on;
for i = 1 : 1 : k
    param = y_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    d = polyval(param, t);
    plot(t, d);
end
hold off;
grid on;

figure(7)
title('velocity of y');
xlabel('t [s]');
ylabel('velocity [m/s]');
hold on;
for i = 1 : 1 : k
    param = y_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    v = polyval(param, t);
    plot(t, v);
end
hold off;
grid on;

figure(8)
title('acceleration of y');
xlabel('t [s]');
ylabel('acceleration [m/s^2]');
hold on;
for i = 1 : 1 : k
    param = y_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    param = polyder(param);
    a = polyval(param, t);
    plot(t, a);
end
hold off;
grid on;

figure(9)
title('jerk of y');
xlabel('t [s]');
ylabel('jerk [m/s^3]');
hold on;
for i = 1 : 1 : k
    param = y_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    param = polyder(param);
    param = polyder(param);
    j = polyval(param, t);
    plot(t, j);
end
hold off;
grid on;

figure(10)
title('snap of y');
xlabel('t [s]');
ylabel('snap [m/s^4]');
hold on;
for i = 1 : 1 : k
    param = y_param(1+(i-1)*(n+1) : n+1+(i-1)*(n+1))';
    param = fliplr(param);
    t0 = t_arr(i);
    t1 = t_arr(i+1);
    t = [t0: 0.001 : t1];
    param = polyder(param);
    param = polyder(param);
    param = polyder(param);
    param = polyder(param);
    s = polyval(param, t);
    plot(t, s);
end
hold off;
grid on;


end