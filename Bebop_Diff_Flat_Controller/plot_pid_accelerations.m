function plot_pid_accelerations(params_plot, i)
    figure(i)
    t= 1:100;
    hold on
    plot(t, params_plot.accelerations(t, 1));
    plot(t, params_plot.accelerations(t, 2));
    plot(t, params_plot.accelerations(t, 3));
    title("Calculated Desired Input Accelerations")
    legend("acc_1", "acc_2", "acc_3", "LineWidth", 3);
    hold off
    
end