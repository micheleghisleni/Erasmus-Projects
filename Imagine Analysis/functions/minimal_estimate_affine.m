function [A, t] = minimal_estimate_affine(pts, pts_tilde)
    
    % Extract x,y coordinates
    x_tilde = pts_tilde(1,:);
    y_tilde = pts_tilde(2,:);
    x = pts(1,:);
    y = pts(2,:);
    
% Closed-form solution
    M = [x(1)   y(1)    0      0        1      0;
         0      0       x(1)   y(1)     0      1;
         x(2)   y(2)    0      0        1      0;
         0      0       x(2)   y(2)     0      1;
         x(3)   y(3)    0      0        1      0;
         0      0       x(3)   y(3)     0      1];
  
    v = [x_tilde(1);
         y_tilde(1);
         x_tilde(2);
         y_tilde(2);
         x_tilde(3);
         y_tilde(3)];

%%%  M*sigma = v %%%

 sigma = M\v;
 A = [sigma(1) sigma(2);
      sigma(3) sigma(4)];
 t = [sigma(5);sigma(6)];

end