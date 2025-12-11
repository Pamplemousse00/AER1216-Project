function [final_path, final_yaw] = plan_path(wait_time)
% Execute the path planning algorithm for each part of the task
% outputs:
% final_path: final path for the whole task
% final_yaw: final yaw path for the whole task

% init
% init_z = 121.92;
init_z = 0;
hover_z = -30.48;
hover_y = 0;
yaw = 0;  % up north

init_n_steps = 1000;
init_x_list = linspace(0, 0, init_n_steps)';
init_y_list = linspace(0, 0, init_n_steps)';
init_z_list = linspace(init_z, hover_z, init_n_steps)';
init_path = [init_y_list, init_x_list, init_z_list];
init_yaw_path = linspace(0, 0, init_n_steps)';

% hover
hover_n_steps = 100;
hover_x_list = linspace(0, 0, hover_n_steps)';
hover_y_list = linspace(hover_y, hover_y, hover_n_steps)';
hover_z_list = linspace(hover_z, hover_z, hover_n_steps)';
hover_path = [hover_y_list, hover_x_list, hover_z_list];
hover_yaw_path = linspace(0, 0, hover_n_steps)';

% traj_1
traj_1_start_x = 0;
traj_z = -30.48;
traj_1_n_steps = 800 / wait_time;
traj_1_end_x = traj_1_start_x - traj_1_n_steps;
traj_1_x_list = linspace(traj_1_start_x, traj_1_end_x, traj_1_n_steps)';
traj_1_y_list = linspace(0, 0, traj_1_n_steps)';
traj_1_z_list = linspace(traj_z, traj_z, traj_1_n_steps)';
traj_1_path = [traj_1_y_list, traj_1_x_list, traj_1_z_list];
traj_1_yaw_path = linspace(pi / 2, pi / 2, traj_1_n_steps)';

% traj 2
traj_2_start_x = traj_1_end_x;
traj_2_radius = 400 / wait_time;
traj_2_n_steps = round(traj_2_radius * pi * 10);
traj_2_space = linspace(pi / 2, 3 * pi / 2, traj_2_n_steps);
traj_2_x_list = (traj_2_radius / 2 * cos(traj_2_space) + traj_2_start_x)';
traj_2_y_list = (traj_2_radius / 2 * sin(traj_2_space) - traj_2_radius / 2)';
traj_2_z_list = ones(traj_2_n_steps, 1) * traj_z;
traj_2_path = [traj_2_y_list, traj_2_x_list, traj_2_z_list];
traj_2_yaw_path = linspace(pi / 2, 3 * pi / 2, traj_2_n_steps)';

% return 1
return_1_n_steps = round(200 / wait_time * 2^0.5);
return_1_x_list = linspace(traj_2_start_x, traj_2_start_x + 200 / wait_time, return_1_n_steps)';
return_1_y_list = linspace(hover_y - 400 / wait_time, hover_y - 200 / wait_time, return_1_n_steps)';
return_1_z_list = linspace(traj_z, traj_z, return_1_n_steps)';
return_1_path = [return_1_y_list, return_1_x_list, return_1_z_list];
return_1_yaw_path = linspace(7 * pi / 4, 7 * pi / 4, return_1_n_steps)';

% return 2
return_2_n_steps = 400 / wait_time;
return_2_x_list = linspace(traj_2_start_x + 200 / wait_time, traj_2_start_x + 600 / wait_time, return_2_n_steps)';
return_2_y_list = linspace(hover_y - 200 / wait_time, hover_y - 200 / wait_time, return_2_n_steps)';
return_2_z_list = linspace(traj_z, traj_z, return_2_n_steps)';
return_2_path = [return_2_y_list, return_2_x_list, return_2_z_list];
return_2_yaw_path = linspace(3 * pi / 2, 3 * pi / 2, return_2_n_steps)';

% return 3
return_3_n_steps = round(200 / wait_time * 2^0.5);
return_3_x_list = linspace(traj_2_start_x + 600 / wait_time, traj_2_start_x + 800 / wait_time, return_3_n_steps)';
return_3_y_list = linspace(hover_y - 200 / wait_time, hover_y, return_3_n_steps)';
return_3_z_list = linspace(traj_z, traj_z, return_3_n_steps)';
return_3_path = [return_3_y_list, return_3_x_list, return_3_z_list];
return_3_yaw_path = linspace(7 * pi / 4, 7 * pi / 4, return_3_n_steps)';

% land
land_n_steps = 1000;
land_x_list = linspace(0, 0, land_n_steps)';
land_y_list = linspace(0, 0, land_n_steps)';
land_z_list = linspace(traj_z, init_z, land_n_steps)';
land_path = [land_y_list, land_x_list, land_z_list];
land_yaw_path = linspace(7 * pi / 4, 2 * pi, land_n_steps)';

% total_path
final_path = [init_path; hover_path; traj_1_path; traj_2_path; return_1_path; return_2_path; return_3_path; land_path];
final_yaw = [init_yaw_path; hover_yaw_path; traj_1_yaw_path; traj_2_yaw_path; return_1_yaw_path; return_2_yaw_path; return_3_yaw_path; land_yaw_path];
end