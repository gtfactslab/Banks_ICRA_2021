function plot_control_input(params_plot, i, j)
    figure(i)
    t= 1:j;
    plot(t, params_plot.moments(t, 1));
    title("Thrust")
    
    figure(i+1)
    hold on
    plot(t, params_plot.moments(t, 2), 'LineWidth', 2);
    plot(t, params_plot.moments(t, 3), 'LineWidth', 2);
    plot(t, params_plot.moments(t, 4), 'LineWidth', 2);
    hold off
    title("Torques")
    legend("t_x", "t_y", "t_z");
    hold off
end