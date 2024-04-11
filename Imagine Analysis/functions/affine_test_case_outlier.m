function [pts, pts_tilde, A_true, t_true] = affine_test_case_outlier(outlier_rate,std)
    
    % Inizialize the vector of outlier points transformed. 
        % Usefull for outlier ratio = 0 / inlier ratio = 1
    pts_outlier_transf = double.empty(2,0);
    
    % Parameters
    N_points = 1000;
    size = 640*480;

    % Generate N random points with x and y coordinates
    pts_x = randi([-640 640],[1,N_points]);
    pts_y = randi([-480 480],[1,N_points]);
    pts = [pts_x;pts_y];

    % Generate inliers from true transform
    N_inlier = round((1-outlier_rate)*N_points);
    pts_inlier = pts(:,1:N_inlier);
    A_true = randi([-10, 10],2,2);
    t_true_x = randi([-640, 640],1,1);
    t_true_y = randi([-480, 480],1,1);
    t_true = [t_true_x ; t_true_y];
    
    % Generate Gaussian noise using the std given as input
    noise_inlier = std * randn(2,N_inlier);
    
    % Generate transformed points: inlier case
    pts_transf = A_true*pts_inlier + t_true + noise_inlier;

    
    % Generate outliers from random transform for each point
    pts_outlier = pts(:,(N_inlier+1):N_points);
    for i = 1:N_points-N_inlier
        
        A = randi([-10, 10],2,2);
        t_x = randi([-640, 640],1,1);
        t_y = randi([-480, 480],1,1);
        t = [t_x ; t_y];
    
        % Generate Gaussian noise using the std given as input
        noise_outlier = std * randn(2,N_points-N_inlier);
            
        % Generate transformed points: outlier case
        pts_outlier_transf(:,i) = A*pts_outlier(:,i) + t + noise_outlier(:,i);
    end
    
    % Compose output
    pts_tilde = [pts_transf pts_outlier_transf];
    
    % Shuffle the order of points for increase the randomity
    index = randperm(N_points);
    pts= pts(:,index);
    pts_tilde = pts_tilde(:,index);
end