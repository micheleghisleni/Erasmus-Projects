function [A, t] = least_squares_affine(pts, pts_tilde)
    
    % Extract x,y coordinates
    x_tilde = pts_tilde(1,:);
    y_tilde = pts_tilde(2,:);
    x = pts(1,:);
    y = pts(2,:);
    M = [];
    v = [];
    
    % Compose the matrices for the closed-form solution of the affine
    % transformation
    for i = 1:length(pts)
        M = [M;
             x(i)   y(i)    0      0        1      0;
             0      0       x(i)   y(i)     0      1;];

         v = [v;
              x_tilde(i);
              y_tilde(i);];
    end
    
    % Solve and generate transformation
    sigma = M\v;

    A = [sigma(1) sigma(2);
         sigma(3) sigma(4)];
    t = [sigma(5);sigma(6)];
end