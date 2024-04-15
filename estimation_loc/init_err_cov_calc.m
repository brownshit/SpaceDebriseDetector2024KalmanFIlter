function init_cov = init_err_cov_calc(var,iter_Num)
    %
    %
    tot_iter = 1000;
    dt = 0.01;
    temp_P = zeros(2,2);
    time2 = 0;
    noise_pos = zeros(2,tot_iter);
    real_pos = zeros(2,tot_iter);
    pres_r = zeros(1,tot_iter);

    for ind1 = 1:1:iter_Num
        aver_error = zeros(2,1);
        time1 = 0;
        for ind2 = 1:1:iter_Num
            time1 = time1 + dt;
            [noise_pos(:,ind2),real_pos(:,ind2), pres_r(1,ind2)] ...
                = pos_gen(var,time1);

            %accumulate error
            aver_error(:,1) = aver_error(:,1) + noise_pos(:,ind2)-real_pos(:,ind2);
        end
        aver_error(:,1) = aver_error(:,1)./iter_Num;

        time2 = time2 + dt;
        [noise_pos(:,ind1),real_pos(:,ind1), pres_r(1,ind1)] ...
            = pos_gen(var,time2);

        error = (noise_pos(:,ind1)-real_pos(:,ind1))-aver_error;
        temp_P = temp_P + error*error';
    end
    init_cov = temp_P./iter_Num;
end