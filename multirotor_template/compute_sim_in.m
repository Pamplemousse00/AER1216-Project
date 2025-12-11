function [x_in, y_in, z_in, yaw_in] = compute_sim_in(final_path, final_yaw, wait_time)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
size_path = size(final_path);
len_path = size_path(1);
time_list = zeros(len_path, 1);
x_list = zeros(len_path, 1);
y_list = zeros(len_path, 1);
z_list = zeros(len_path, 1);
for i = 1 : len_path
    time_list(i) = int16((i - 1) * wait_time);
    path_item = final_path(i, :);
    x_list(i) = path_item(1);
    y_list(i) = path_item(2);
    z_list(i) = path_item(3);
end
x_in = timeseries(x_list, time_list);
y_in = timeseries(y_list, time_list);
z_in = timeseries(z_list, time_list);
yaw_in = timeseries(final_yaw, time_list);
end