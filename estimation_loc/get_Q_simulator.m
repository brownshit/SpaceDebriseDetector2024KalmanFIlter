function [Q_mat ,Expect_of_process_noise,ABS_Expect_of_process_noise]...
    = get_Q_simulator(var, E_w, mod)
    format long e;
    %   Q simulation
    %   sigma_square = [1e-3;1e-2;1e-1;1;1e1];
    %
    %   Expect_of_process_noise is 2x1 vec; 
    %   (1,1) : E(wx) / (2,1) : E(wy)
    %var = 1e-2;
    center = [0;0];
    iteration = 1e4;
    tot_iter = 1e3;
    dt = 0.01;
    Q_mat = zeros(2,2);

    noise_pos = zeros(2,tot_iter);
    real_pos = zeros(2,tot_iter);
    %=================================
    PROCESS_NOISE = zeros(2,1);
    process_noise = zeros(2,tot_iter); %[wx;wy];

    process_noise_accumulator = [0;0];
    process_noise_accumulator_ABS = [0;0];
    process_noise_memory = zeros(2,iteration);
    %=================================

    for iter = 1:1:iteration
        time = 0;
        for time_ind = 1:1:tot_iter
            time = time + dt;
            [noise_pos(:,time_ind),real_pos(:,time_ind), ~] ...
                = pos_gen(var,time);
        end
        
        
        %error_memory = zeros(2,end_pnt);
        
        theta = zeros(1,tot_iter);

        for MovPnt = 2:1:tot_iter      %skip first step if we exactly know about init value.
            F = [cos(theta(1,MovPnt-1)),-sin(theta(1,MovPnt-1));...
                sin(theta(1,MovPnt-1)), cos(theta(1,MovPnt-1))];
            
            if MovPnt >= 3      
                process_noise(:,MovPnt) = ...
                    real_pos(:,MovPnt)...
                    -F*noise_pos(:,MovPnt-1)...
                    -E_w(:,1);
            end
            
            r_k     = sqrt((noise_pos(1,MovPnt)-center(1,1))^2+(noise_pos(2,MovPnt)-center(2,1))^2);
            r_k_1   = sqrt((noise_pos(1,MovPnt-1)-center(1,1))^2+(noise_pos(2,MovPnt-1)-center(2,1))^2);
            l_k     = sqrt((noise_pos(1,MovPnt)-noise_pos(1,MovPnt-1))^2+(noise_pos(2,MovPnt)-noise_pos(2,MovPnt-1))^2);
            value = (r_k^2+r_k_1^2-l_k^2)/(2*r_k*r_k_1);
            theta(1,MovPnt) = acos(value);
        end
        
        % I took the max value of each process noise in one procedure.
        % which could make our filter more robust.
        

        %option #1
        if mod == 1
            PROCESS_NOISE(:,1) = mean(process_noise,2);
        end

        %option #2
        if mod == 2
            PROCESS_NOISE(:,1) = max(process_noise,[],2);
        end

        %after calc 0~2, we can get process noise -> to compose Q matrix.
        %for [2;2] point, we could get process noise calc step.
        process_noise_accumulator = process_noise_accumulator + PROCESS_NOISE;
        process_noise_accumulator_ABS = process_noise_accumulator_ABS + abs(PROCESS_NOISE);

        %for figure process noise
        process_noise_memory(:,iter) = PROCESS_NOISE;

        %Q_mat accumulation.
        Q_mat = Q_mat + (PROCESS_NOISE)*(PROCESS_NOISE)';
    end
    %calc expectation of process noise.
    Expect_of_process_noise = process_noise_accumulator./iteration;
    ABS_Expect_of_process_noise = process_noise_accumulator_ABS./iteration;
    Q_mat = Q_mat./iteration;

    %=================================
    %{
    figure;
    bivariate_hist(process_noise_memory(1,:),process_noise_memory(2,:), 0.01);
    xlabel('[Wx]');
    ylabel('[Wy]');
    title('[process noise]');

    %Q_matrix Expectation
    
    disp("[E(wx)] : "+Expect_of_process_noise(1,1)+" [E(wy)] : "+Expect_of_process_noise(2,1));
    disp("we got Q matrix");
    disp(Q_mat);


    %}
end