function plot_vels(params_plot, i,j)
    figure(i)
    t= 1:j;
    hold on
    plot(t, params_plot.input_vector(t, 1));

    plot(t, params_plot.input_vector(t, 2));

    plot(t, params_plot.input_vector(t, 3));
    
    plot(t, params_plot.input_vector(t, 4));
    
    
    title("Motor Speeds")
    legend('w_1', 'w_2', 'w_3', 'w_4', 'LineWidth', 3);
    hold off
end