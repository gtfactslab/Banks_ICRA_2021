function plot_hover_input(params_plot, i, j)
    figure(i)
    t= 1:j;
    plot(t, params_plot.angles_n_thrust(t, 1));
    title("Thrust")
    
    figure(i+1)
    hold on
    plot(t, params_plot.angles_n_thrust(t, 2));
    plot(t, params_plot.angles_n_thrust(t, 3));
    plot(t, params_plot.angles_n_thrust(t, 4));
    title("Input Angles")
    legend("Roll", "Pitch", "Yaw", "LineWidth", 3);
    hold off
    
end