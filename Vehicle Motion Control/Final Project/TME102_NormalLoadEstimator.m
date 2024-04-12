WC_r_x = 1.075;
WC_f_x = 4.059;
h = 0.678;
h_rc = .154;
sensor_x = 2.8827;
m = 2296;
g = 9.81;
w_f = 1.676;
w_r = 1.668;
J_x = 1204.674;
J_z = 4027.903;
M_z = 0;
C_f = 1.5*(6200-4000)/50;
C_r = (4900-3800)/25;

l_r = sensor_x-WC_r_x;
l_f = WC_f_x-sensor_x;

F_1z = zeros(size(Time));
F_2z = zeros(size(Time));
F_3z = zeros(size(Time));
F_4z = zeros(size(Time));

az_n=az+9.81;

for i=1:size(Time)
    F_1z(i) = (l_r*m*az_n(i) - m*h*ax(i))/(2*(l_f+l_r)) - m*ay(i)*(h*C_f*(l_f+l_r) - h_rc*(C_f*l_f-C_r*l_r))/(w_f*(l_r+l_f)*(C_f+C_r)) + C_f*J_x*rollAcc(i)/(w_f*(C_f+C_r)) - (h_rc*J_z*yawAcc(i) + h_rc*M_z)/(w_f*(l_f+l_r));
    F_2z(i) = (l_r*m*az_n(i) - m*h*ax(i))/(2*(l_f+l_r)) + m*ay(i)*(h*C_f*(l_f+l_r) - h_rc*(C_f*l_f-C_r*l_r))/(w_f*(l_r+l_f)*(C_f+C_r)) - C_f*J_x*rollAcc(i)/(w_f*(C_f+C_r)) + (h_rc*J_z*yawAcc(i) - h_rc*M_z)/(w_f*(l_f+l_r));
    F_3z(i) = (l_f*m*az_n(i) + m*h*ax(i))/(2*(l_f+l_r)) - m*ay(i)*(h*C_f*(l_f+l_r) + h_rc*(C_f*l_f-C_r*l_r))/(w_r*(l_r+l_f)*(C_f+C_r)) + C_f*J_x*rollAcc(i)/(w_r*(C_f+C_r)) + (h_rc*J_z*yawAcc(i) - h_rc*M_z)/(w_r*(l_f+l_r));
    F_4z(i) = (l_f*m*az_n(i) + m*h*ax(i))/(2*(l_f+l_r)) + m*ay(i)*(h*C_f*(l_f+l_r) + h_rc*(C_f*l_f-C_r*l_r))/(w_r*(l_r+l_f)*(C_f+C_r)) - C_f*J_x*rollAcc(i)/(w_r*(C_f+C_r)) - (h_rc*J_z*yawAcc(i) + h_rc*M_z)/(w_r*(l_f+l_r));
end

figure
subplot(221)
plot(Time,F_1z,Time,FzFL)
subplot(222)
plot(Time,F_2z,Time,FzFR)
subplot(223)
plot(Time,F_3z,Time,FzRL)
subplot(224)
plot(Time,F_4z,Time,FzRR)

%RMS = sqrt(1/size(Time)*sum(F_1z