v_kph = 80;

v_x = v_kph/3.6;
C_f = 1.22e+05;
C_r = 9.16e+04;
l_f = 4.06-2.88;
l_r = 2.88-1.08;
m = 2297;
J = 3183;

a11 = (C_f+C_r)/abs(v_x);
a12 = (C_f*l_f-C_r*l_r)/abs(v_x) + m*v_x;
a21 = (C_f*l_f-C_r*l_r)/abs(v_x);
a22 = (C_f*l_f^2 + C_r*l_r^2)/abs(v_x);

A = -inv([m 0;0 J])*[a11 a12;a21 a22];
B =  inv([m 0;0 J])*[C_f;C_f*l_f];
C = [1 0;0 1];
D = [0;0];

sys = ss(A,B,C,D);
[num,den] = ss2tf(A,B,C,D);

s = tf('s');

ot_yawRate = tf(num(2,:),den)

bodemag(ot_yawRate,{1,20}); grid;