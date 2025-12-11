function [attitude_dot] = attitude_compdyn(attitude, ang_vel)
phi = attitude(1);
theta = attitude(2);

S_B_inv = [1, sin(phi) * tan(theta), cos(phi) * tan(theta); 0, cos(phi), -sin(phi); 0, sin(phi) * sec(theta), cos(phi) * sec(theta)];
attitude_dot = S_B_inv * ang_vel;
end 



