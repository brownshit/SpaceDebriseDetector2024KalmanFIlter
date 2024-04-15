close all;
center = [0;0];
dt = 0.01;
sig_sq = [0.001, 0.01, 0.1, 1, 10];
tot_iter = 1000;
time = 0;
noise_pos = zeros(2,tot_iter);
real_pos = zeros(2,tot_iter);

% Q value and expectation.
Q_max = zeros(2,2,5);
fileID = fopen('Q_max.txt', 'r');
Q_raw = fscanf(fileID, '%f');
fclose(fileID);
for i1=1:1:size(sig_sq,1)
    for i2=1:1:2
        for i22=1:1:2
           Q_max(i2,i22,i1) = Q_raw((i1-1)*4+(i2-1)*2+i22,1);
        end
    end
end

E_w_max = zeros(2,5);
fileID = fopen('E_w_max.txt', 'r');
E_w_raw = fscanf(fileID, '%f');
fclose(fileID);
for i1=1:1:size(sig_sq,1)
    for i2=1:1:2
        E_w_max(i2,i1) = E_w_raw((i1-1)*2+i2,1);
    end
end

% EKF estimation of position.
esti_pos = zeros(2,tot_iter);
cov_esti = zeros(2,2,tot_iter);

theta = zeros(1,tot_iter);
angular_vel = zeros(1,tot_iter);

pres_r = zeros(1,tot_iter);

for time_ind = 1:1:tot_iter
    time = time + dt;
    [noise_pos(:,time_ind),real_pos(:,time_ind), pres_r(1,time_ind)] = pos_gen(sig_sq(1,2),time);
end

%USE EKF for position.
%for step 1

for pnt_ind = 2:1:tot_iter
    %EKF pos
    if pnt_ind > 3
        [esti_pos(:,pnt_ind),cov_esti(:,:,pnt_ind)] = ...
            pos_Filter...
            (esti_pos(:,pnt_ind-1),cov_esti(:,:,pnt_ind-1),...
            sig_sq(1,2),noise_pos(:,pnt_ind),...
            Q_max(:,:,2),zeros(2,1),...%E_w_max(:,2),...
            theta(1,pnt_ind-1));
    else
        %for step 2,3 : use observation as estimation value.
        esti_pos(:,pnt_ind) = noise_pos(:,pnt_ind);
        if pnt_ind == 3
            cov_esti(:,:,pnt_ind) = init_err_cov_calc(sig_sq(1,2),tot_iter/10);        
        end
    end
    
    %theta numerically.
    r_k     = sqrt((esti_pos(1,pnt_ind)-center(1,1))^2+(esti_pos(2,pnt_ind)-center(2,1))^2);
    r_k_1   = sqrt((esti_pos(1,pnt_ind-1)-center(1,1))^2+(esti_pos(2,pnt_ind-1)-center(2,1))^2);
    l_k     = sqrt((esti_pos(1,pnt_ind)-esti_pos(1,pnt_ind-1))^2+(esti_pos(2,pnt_ind)-esti_pos(2,pnt_ind-1))^2);
    value = (r_k^2+r_k_1^2-l_k^2)/(2*r_k*r_k_1);
    theta(1,pnt_ind) = acos(value);
    angular_vel(1,pnt_ind) = theta(1,pnt_ind)/dt;
end
%{

%}

time = 1:1:tot_iter;
str_ind = 1;
end_ind = tot_iter/4;
for i =1:1:4
    figure;
    titles = "Tracking t = ["+str_ind+"~"+end_ind+"]";
    title(titles);
    plot3(noise_pos(1,str_ind:end_ind),noise_pos(2,str_ind:end_ind),time(str_ind:end_ind),'k*:');
    hold on
    plot3(real_pos(1,str_ind:end_ind),real_pos(2,str_ind:end_ind),time(str_ind:end_ind),'o');
    plot3(esti_pos(1,str_ind:end_ind),esti_pos(2,str_ind:end_ind),time(str_ind:end_ind),'*--');
    hold off
    %axis equal
    legend('Observation','Real Position','Filtered position');

    str_ind = str_ind + tot_iter/4;
    end_ind = end_ind + tot_iter/4;
end

%Error Tracking
time = 1:1:tot_iter;
str_ind = 1;
end_ind = tot_iter/4;
error_noi_pos = (noise_pos-real_pos).^2;
error_filter = (esti_pos-real_pos).^2;

ErrNoi = zeros(1,tot_iter);
ErrFil = zeros(1,tot_iter);
for ind = 1:1:tot_iter
    ErrNoi(ind) = error_noi_pos(1,ind)^2+error_noi_pos(2,ind)^2;
    ErrFil(ind) = error_filter(1,ind)^2+error_filter(2,ind)^2;
end

for i = 1:1:4
    figure;
    if i == 1
        str_ind = 10;
    end
    plot(time(str_ind:end_ind),ErrNoi(str_ind:end_ind));
    hold on
    plot(time(str_ind:end_ind),ErrFil(str_ind:end_ind));
    hold off
    title('Mean Square Error by Section')
    xlabel('iteration');
    ylabel('Error');
    legend('Observation Error','Filtered Error');
    axis on

    str_ind = str_ind + tot_iter/4;
    end_ind = end_ind + tot_iter/4;
end

%{

%}