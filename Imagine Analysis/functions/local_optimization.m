function [A_LO,t_LO,outliers_LO] = local_optimization(pts,pts_tilde,threshold,reps)
    
    % Parameters
    N_sample = min(size(pts,2),12);
    outliers_LO= size(pts,2);

    % Inner iterations of Local Optimization
    for i = 1:reps
        
        % Non-minimal sample S from inliers I of M_new_best
        indx_sample = randsample(size(pts,2),N_sample);
        pts_LO = pts(:,indx_sample);
        pts_tilde_LO = pts_tilde(:,indx_sample);
    
        % New transformation estimate using least squares affine
        [A_temp,t_temp] = least_squares_affine(pts_LO,pts_tilde_LO);
        res = residual(A_temp, t_temp, pts, pts_tilde);
        N_outliers = sum(res > threshold);
        
        if N_outliers < outliers_LO
            A_LO = A_temp;
            t_LO = t_temp;
            outliers_LO = N_outliers;
        end
    end
end