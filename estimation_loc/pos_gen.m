function [noise_pos,real_pos,pres_r] = pos_gen(var,time_ind)
    r = 10;
    w = 10;
    pres_r = r*(1-exp(-20*time_ind));
    x = pres_r*cos(2*pi*w*time_ind) + sqrt(var)*randn;
    y = pres_r*sin(2*pi*w*time_ind) + sqrt(var)*randn;
    
    act_x = pres_r*cos(2*pi*w*time_ind);
    act_y = pres_r*sin(2*pi*w*time_ind);
    noise_pos = [x;y];
    real_pos = [act_x;act_y];
end