function [A_best,t_best,i_while] = ransac_LO(pts, pts_tilde, threshold,epsilon,eta,test_correspondances)
    
    %Underestimation of epsilon for avoid too few iterations
    if epsilon > 0.6
        epsilon = 0.6;
    end
    
    % Initialization of A and t to avoid the case when the first if (line 36) will never be true.
    index = randperm(size(pts,2), 3);
    pts_est = pts(:,index);
    pts_tilde_est = pts_tilde(:,index);
    [A_best, t_best] = minimal_estimate_affine(pts_est, pts_tilde_est);
    
    % Initialize variables for RANSAC algorithm
    i_while = 0;
    reps = 10;
    N_outliers_min = size(pts,2);
    N_outliers_min_LO = size(pts,2);
    N_iterations = ceil(log(eta)/log(1-epsilon^(3+test_correspondances)));

    % Beginning of the RANSAC
    while (i_while < N_iterations)
        
        % Estimate transformation for minimal subset of points ( 3 )
        index = randperm(size(pts,2), 3);
        pts_est = pts(:,index);
        pts_tilde_est = pts_tilde(:,index);
        [A_temp, t_temp] = minimal_estimate_affine(pts_est, pts_tilde_est);
        
        % Randomly select d correspondences
        index = randsample(size(pts,2),test_correspondances);
        pts_sample = pts(:,index);
        pts_tilde_sample = pts_tilde(:,index);
        
        % Count outliers in the d correspondences
        res_d = residual(A_temp, t_temp, pts_sample, pts_tilde_sample);
        N_ouliers_d = sum(res_d > threshold);

        % Evaluating the model just if the d correspondeces are all inliers 
        if N_ouliers_d == 0
            
            % Evaluating the model on all the points
            res_all = residual(A_temp, t_temp, pts, pts_tilde);
            N_outliers_all = sum(res_all > threshold);
            
            if N_outliers_all < N_outliers_min
                
                % Update minimum and best estimate from minimal sample
                N_outliers_min = N_outliers_all;
                A_new_best = A_temp;
                t_new_best = t_temp;

                I = pts(:,res_all < threshold);
                I_tilde = pts_tilde(:,res_all < threshold);

                % Local Optimization
                [A_LO, t_LO, N_outliers_LO] = local_optimization(I, I_tilde,threshold,reps);

                if N_outliers_LO < N_outliers_min_LO
               
                    N_outliers_min_LO = N_outliers_LO;
                    A_best = A_LO;
                    t_best = t_LO;
               
                    % Update loop variables
                    N_inliers_max = size(pts,2) - N_outliers_min_LO;
                    epsilon = N_inliers_max/size(pts,2);
                    N_iterations = ceil(log(eta)/log(1-epsilon^(3+test_correspondances)));
                end
            end
        end
        i_while = i_while + 1;
    end   
end