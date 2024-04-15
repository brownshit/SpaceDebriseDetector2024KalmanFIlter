function bivariate_hist(data_x, data_y, bin_width)
    %figure;
    histogram2(data_x, data_y, 'BinWidth', [bin_width, bin_width]);
end