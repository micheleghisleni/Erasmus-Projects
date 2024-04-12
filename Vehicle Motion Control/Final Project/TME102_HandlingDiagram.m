l =2.984;

R = vx.^2./ay;
aynorm_et = ay/9.81;

SteerAng = (SteerAng_FL+SteerAng_FR)/2;

sideslipbal_et = -180/pi*(SteerAng+l./R);


figure
subplot(111)
plot(sideslipbal,aynorm,sideslipbal_e,aynorm_e,sideslipbal_et,aynorm_et)
legend("ICE","Electric","Adjusted Electric")
xlabel("-Steer Angle + 1/R")
ylabel("Normalized Lateral Acceleration")