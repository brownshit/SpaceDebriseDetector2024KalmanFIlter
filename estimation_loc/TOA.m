function pos = TOA(measu_dist)
    %
    %
    %{
    %}
    %위치측위 알고리즘 동작.
    %basic settings.
    z = zeros(2,1);
    H = zeros(2,2);

    ankx = 630; anky = 540;
    Anchor_1 = [0;0];
    Anchor_2 = [ankx;0];
    Anchor_3 = [0;anky];

    %for convenience I just simply substitude 1 into mov_pnt
    mov_pnt = 1;
    z(1,1) = (measu_dist(2,mov_pnt)^2-measu_dist(1,mov_pnt)^2)+...
        (Anchor_1(1,1)^2+Anchor_1(2,1)^2)-...
        (Anchor_2(1,1)^2+Anchor_2(2,1)^2);

    z(2,1) = (measu_dist(3,mov_pnt)^2-measu_dist(1,mov_pnt)^2)+...
        (Anchor_1(1,1)^2+Anchor_1(2,1)^2)-...
        (Anchor_3(1,1)^2+Anchor_3(2,1)^2);

%{
    z(3,1) = (measu_dist(3,mov_pnt)^2-measu_dist(2,mov_pnt)^2)+...
        (Anchor_2(1,1)^2+Anchor_2(2,1)^2)-...
        (Anchor_3(1,1)^2+Anchor_3(2,1)^2);
%}

    H(1,:) = 2*[(Anchor_1(1,1)-Anchor_2(1,1)),...
        (Anchor_1(2,1)-Anchor_2(2,1))];

    H(2,:) = 2*[(Anchor_1(1,1)-Anchor_3(1,1)),...
        (Anchor_1(2,1)-Anchor_3(2,1))];
                
%{
    H(3,:) = 2*[(Anchor_2(1,1)-Anchor_3(1,1)),...
        (Anchor_2(2,1)-Anchor_3(2,1))];
%}
    
    pos = (inv(H'*H))*H'*z;      %Left pseudo inv
end