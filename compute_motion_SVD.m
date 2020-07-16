% Date created: Nov-19
% Author: Miller

% This is the core function that computes motion from two pointclouds
function [C, r] = compute_motion_SVD( p1, p2 )

    % ------insert your motion-from-two-pointclouds algorithm here------
    
    % ASSUME: All weights are one
    
    % calculate the centoid of the point cloud 1
    count = 0;
    center_1 = zeros(3,1);
    for i = 1 : size(p1,2)
        center_1 = center_1 + p1(:,i);
        count = count + 1;
    end
    center_1 = center_1 ./ count;
    
    % calculate the centoid of the point cloud 2
    count = 0;
    center_2 = zeros(3,1);
    for i = 1 : size(p2,2)
        center_2 = center_2 + p2(:,i);
        count = count + 1;
    end
    center_2 = center_2 ./ count;
    
    % Calculate the W matrix 3 by 3 matrix
    % loop through every pair
    W = zeros(3,3);
    count = 0;
    for i = 1 : size(p1,2)
        W = W + ( p2(:,i) - center_2 ) * ( p1(:,i) - center_1)';
        count = count + 1;
    end
    W = W ./ count;
    
    [U, ~, V] = svd(W);
    
    C = U * [1 0 0; 0 1 0 ; 0 0 det(U)*det(V)] * V';     % temporary to make script run
    r = - C' * center_2 + center_1; % temporary to make script run
   
    
    
    % ------end of your motion-from-two-pointclouds algorithm-------
    
end