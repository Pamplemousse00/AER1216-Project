function [] = plot_path_2d(path_list, x_out, y_out, x_bound, y_bound)
% Plot the path in green as well as obstacles in red and landmarks in blue dots
% inputs:
% path_list: list of coords of points in the path

figure
x_list = path_list(:, 1);
y_list = path_list(:, 2);
plot(x_list, y_list, 'g', 'LineWidth', 3);
hold on

x_out_data = x_out.Data;
y_out_data = y_out.Data;
plot(x_out_data', y_out_data', 'm', 'LineWidth', 3);
hold on

xlim(x_bound)
ylim(y_bound)
title("Simulation Results 2D")
xlabel("x (1/10 m)")
ylabel("y (1/10 m)")
hold off
end