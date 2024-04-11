function [n_inliers,avg_res]=residual_evaluation(A,t,pts,pts_tilde,threshold)
% Same step of "residual" functions
pts_transformed = A*pts + t;
error = pts_transformed - pts_tilde;
distance_sqrd = sum(error.^2, 1);
res = sqrt(distance_sqrd);

% Evaluate all the inliers
res_inliers = res(res<threshold);
% Evaluate the number of inliers
n_inliers = sum(res<threshold);
% Evaluate the average residual length between all the inliers
avg_res = sum(res_inliers) / n_inliers;
end
