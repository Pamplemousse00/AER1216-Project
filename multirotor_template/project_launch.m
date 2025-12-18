clear all
close all

%% PATH PLANNING %% 
wait_time = 0.2;  % TODO: Please adjust it to set speed if necessary

x_bound = [-1500 / wait_time, 50];
y_bound = [-1000 / wait_time, 120];
z_bound = [-50, 0];

[final_path, final_yaw] = plan_path(wait_time);

%% MODEL LAUNCH %%
[x_in, y_in, z_in, yaw_in] = compute_sim_in(final_path, final_yaw, wait_time);


Jr = 1e-4; 
Ixx = 0.03;  % moment of inertia at x-axis
Iyy = 0.03;  % moment of inertia at y-axis
Izz = 0.06;  % moment of inertia at z-axis
I_mat = diag([Ixx Iyy Izz]);
L = 0.225;          
k = 0.09;           
b = 0.07;     
m = 1.6;

%% Parameters
g = 9.81;

sim('Quadrotor_Model_original.slx');

fprintf("Simulation finished")

%% PLOT RESULT %%
plot_path_2d(final_path, x_out, y_out, x_bound, y_bound);
plot_path_3d(final_path, x_out, y_out, z_out, x_bound, y_bound, z_bound);