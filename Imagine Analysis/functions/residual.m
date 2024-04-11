function res = residual(A, t, pts, pts_tilde)
    
    % Transform points
    pts_transformed = A*pts + t;
    % Evaluate the error
    error = pts_transformed - pts_tilde;
    distance_sqrt = sum(error.^2, 1);
    % Evaluate residual length
    res = sqrt(distance_sqrt);
end