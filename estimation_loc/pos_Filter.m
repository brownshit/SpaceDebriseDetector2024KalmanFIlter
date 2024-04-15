function [esti_pos, esti_cov] = ...
    pos_Filter(esti_pos_prev, esti_cov_prev,...
    noise_var, z, Q, E_w,...
    prev_theta)
    %this function is for Kalman Filter

    %codes implementation.
    %init_pos = [0;0];
    dt = 0.01;
    F = [cos(prev_theta),-sin(prev_theta);sin(prev_theta),cos(prev_theta)];  %for modified state ttransient matrix.
    
    H = eye(2);
    R = eye(2).*sqrt(noise_var);
    %should be renewal by each step.

   
    %%
    %prediction step
    %use modified term.
    pred_pos = F*esti_pos_prev + E_w;

    %%
    %pred error cov mat
    pred_cov = F*esti_cov_prev*F' + Q;  %Q : modified Q(zero-mean)

    %%
    %calc Kalman Gain
    Kalman_Gain = pred_cov*H'*pinv(H*pred_cov*H'+R);

    %%
    %estiamtion step
    esti_pos = pred_pos+Kalman_Gain*(z-H*pred_pos);
    esti_cov = pred_cov-Kalman_Gain*H*pred_cov;
end