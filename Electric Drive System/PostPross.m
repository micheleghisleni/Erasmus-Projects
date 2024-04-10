close all

%**********************************************************************
% Induction Machine operation plot                                    *
% stator voltage and currents, speed and dq-currents                  *
%**********************************************************************
figure('Name','Induction Machine')
subplot(2,3,1)
plot(time,usa,'b',time,usb,'r',time,usc,'g')
grid on
xlabel('Time (s)')
ylabel('Stator voltage [V]')
title('blue A-phase, red B-phase, green C-phase')
subplot(2,3,2)
plot(time,Wr*30/pi,'k')
grid on
xlabel('Time (s)')
ylabel('Speed \Omega_r [RPM]')
subplot(2,3,3)
plot(time,PsiR,'k')
grid on
xlabel('Time (s)')
ylabel('Rotor flux \Psi_R (Wb)')
subplot(2,3,4)
plot(time,isa,'b',time,isb,'r',time,isc,'g')
grid on
xlabel('Time (s)')
ylabel('Stator current [A]')
title('blue A-phase, red B-phase, green C-phase')
subplot(2,3,5)
plot(time,real(idq),'b',time,imag(idq),'r')
grid on
xlabel('Time (s)')
ylabel('Stator current [A]')
title('blue d-current, red q-current')
subplot(2,3,6)
plot(time,Te,'b',time,Tload,'r')
grid on
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('blue Te, red Tload')

%**********************************************************************
% figure for the 3-phase converter and modulator                      *
%**********************************************************************
figure('Name','3-phase converter and modulator')
subplot(2,3,1)
plot(time,da,'b',time,db,'r',time,dc,'g',time,tri,'m',time,sa,'b--',time,sb,'r--',time,sc,'g--')
grid on
ylim([-0.1 1.1])
xlabel('Time [s]')
title('d_a blue, d_b red, d_c green, carrier magenta, s_a, s_b, s_c dashed')
subplot(2,3,4)
H=plot(time,usa,'b',time,usb,'r',time,usc,'g',time_dis,uaref,'m',time_dis,ubref,'c',time_dis,ucref,'k',time,Vdc/2,'y',time,-Vdc/2,'y',time_dis,u0,'k');
set(H(9),'Color',[0.5,0.5,0.5]);
grid on
xlabel('Time [s]')
ylabel('Stator voltage [V]')
title('u_s_a blue, u_s_b red, u_s_c green, u_a_r_e_f m,  u_b_r_e_f c,  u_c_r_e_f k, +/-v_d_c/2 y, u_0 grey')
subplot(2,3,2)
plot(time,isa,'b',time,isb,'r',time,isc,'g')
grid on
xlabel('Time [s]')
ylabel('Stator current [A]')
title('i_s_a blue, i_s_b red, i_s_c green')
subplot(2,3,5)
plot(time,usa.*isa+usb.*isb+usc.*isc,'b',time,Vdc.*idc,'r--',time,Vbatt*ibatt,'g')
grid on
xlabel('Time [s]')
ylabel('Power [W]')
title('P_s blue, P_d_c red, P_b_a_t_t green')
subplot(2,3,3)
plot(time,idc,'b',time,ibatt,'r')
grid on
xlabel('Time [s]')
ylabel('Current [A]')
title('i_d_c blue, i_b_a_t_t red')
subplot(2,3,6)
plot(time,Vdc,'b',time,Vbatt+time*0,'r')
grid on
xlabel('Time [s]')
ylabel('Voltage [V]')
title('v_d_c blue, V_b_a_t_t red')


%**********************************************************************
% Current model flux observer                                         *
%**********************************************************************

temp_cont = find(time > 2.2);
temp_dis = find(time_dis > 2.2);
idq_avg = mean(idq(temp_cont));

isab_fundamental = idq_avg*exp(1i*theta(temp_cont));
vl_fundamental = 1i*Lsigma*mean(w1(temp_cont))*idq_avg*exp(1i*theta(temp_cont));
emfab = (RR*real(idq)-RR/LM*PsiR+1i*w1.*PsiR).*exp(1i*theta);
usab = 2/3*(usa+usb*exp(1i*2*pi/3)+usc*exp(1i*4*pi/3));
isab = 2/3*(isa+isb*exp(1i*2*pi/3)+isc*exp(1i*4*pi/3));

figure('Name','Current ripple')
subplot(3,1,1)
plot(time,real(usab),'b',time,imag(usab),'r',time,real(emfab),'g',time,imag(emfab),'m')
grid on
xlabel('Time (s)')
ylabel('Stator voltage  [V]')
title('Blue v_s_,_\alpha, red v_s_,_\beta, d\Psi_R/dt green \alpha comp magenta \beta comp')
%xlim([2.39 2.4])
subplot(3,1,2)
hold on
plot(time,real(usab)-real(emfab)-Rs*real(isab),'b',time,imag(usab)-imag(emfab)-Rs*imag(isab),'r')
plot(time(temp_cont),real(vl_fundamental),'g',time(temp_cont),imag(vl_fundamental),'m')
hold off
grid on
xlabel('Time (s)')
ylabel('inductor voltage [V]')
title('V_L_\sigma = V_s-R_s*i_s-d\Psi_R/dt, Blue \alpha comp red \beta comp, V_L_\sigma_,_f_u_n_d_a_m_e_n_t_a_l \alpha green, \beta magenta')
%xlim([2.39 2.4])
subplot(3,1,3)
hold on
plot(time,real(isab),'b',time,imag(isab),'r',time_dis,real(iabmes),'bo',time_dis,imag(iabmes),'ro')
plot(time(temp_cont),real(isab_fundamental),'g',time(temp_cont),imag(isab_fundamental),'m',time_dis(temp_dis),real(iabmes(temp_dis)),'g--',time_dis(temp_dis),imag(iabmes(temp_dis)),'m--')
hold off
grid on
xlabel('Time (s)')
%xlim([2.39 2.4])
ylabel('Stator current [A]')
title('Blue \alpha, red \beta. Solid line actual current and circles measured')

figure('Name','Current model flux observer')
subplot(2,3,1)
plot(time_dis,thetahat*180/pi,'bo',time,unwrap(theta)*180/pi,'g')
grid on
xlabel('Time (s)')
ylabel('Rotor flux angle (deg)')
title('blue estimated \theta_\Psi_R_,_h_a_t, green actual \theta_\Psi_R')
subplot(2,3,4)
plot(time_dis,(unwrap(theta_dis)-thetahat)*180/pi,'k')
grid on
xlabel('Time (s)')
ylabel('Rotor flux angle error (deg)')
title('\theta_\Psi_R - \theta_\Psi_R_,_h_a_t')
subplot(2,3,2)
plot(time_dis,w1hat/(2*pi),'k',time,w1/(2*pi),'--g')
grid on
xlabel('Time (s)')
ylabel('Stator frequency (Hz)')
title('black estimated \omega_1_,_h_a_t, green actual \omega_1')
subplot(2,3,5)
plot(time,(w1-interp1(time_dis,w1hat,time,'linear','extrap'))/(2*pi),'k')
grid on
xlabel('Time (s)')
ylabel('Stator frequency error (Hz)')
title('\omega_1 - \omega_1_,_h_a_t')
subplot(2,3,3)
plot(time_dis,PsiRref,'--r',time_dis,PsiRhat,'k',time,PsiR,'--g')
grid on
xlabel('Time (s)')
ylabel('Rotor flux (Wb)')
title('black estimated \Psi_R_,_h_a_t, green actual \Psi_R, red reference \Psi_R_,_r_e_f')
subplot(2,3,6)
plot(time_dis,PsiR_dis-PsiRhat,'k')
grid on
xlabel('Time (s)')
ylabel('Rotor flux error (Wb)')
title('\Psi_R - \Psi_R_,_h_a_t')

%**********************************************************************
% Current controller                                                  *
%**********************************************************************
figure('Name','Current controller')
subplot(2,3,1)
plot(time,real(idq),'--g',time_dis,real(idqref),'--r',time_dis,real(idqhat),'k')
grid on
xlabel('Time (s)')
ylabel('i_d (A)')
title('black estimated i_d_,_h_a_t, green actual i_d, red reference i_d_,_r_e_f')
subplot(2,3,2)
plot(time_dis,PsiRref,'--r',time_dis,PsiRhat,'k',time,PsiR,'--g')
grid on
xlabel('Time (s)')
ylabel('Rotor flux (Wb)')
title('black estimated \Psi_R_,_h_a_t, green actual \Psi_R, red reference \Psi_R_,_r_e_f')
subplot(2,3,3)
hold on
plot(time_dis,real(usdq),'b',time_dis,imag(usdq),'r',time_dis,abs(usdq),'g')
plot(time_dis,real(usdqunlim),'b--',time_dis,imag(usdqunlim),'r--',time_dis,abs(usdqunlim),'g--')
hold off
grid on
xlabel('Time (s)')
ylabel('Voltage (V)')
title('blue d, red q, g abs, solid lim, dased unlim')
subplot(2,3,4)
plot(time,imag(idq),'--g',time_dis,imag(idqref),'--r',time_dis,imag(idqhat),'k')
grid on
xlabel('Time (s)')
ylabel('i_q (A)')
title('black estimated i_q_,_h_a_t, green actual i_q, red reference i_q_,_r_e_f')
subplot(2,3,5)
plot(time_dis,Teref,'r',time,Te,'--g',time,Tload,'m--')
grid on
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('green actual T_e, red reference T_e_,_r_e_f, magenta load T_L')
subplot(2,3,6)
plot(time_dis,Wrref*30/pi,'r--',time,Wr*30/pi,'g--')
grid on
xlabel('Time (s)')
ylabel('Rotor Speed (RPM)')
title('green actual \Omega_r, red reference \Omega_r_,_r_e_f')

%**********************************************************************
% Speed controller                                                    *
%**********************************************************************
figure('Name','Speed controller ')
subplot(1,2,1)
plot(time_dis,Wrref*30/pi,'r--',time_dis,Wrmes*30/pi,'ko',time,Wr*30/pi,'g--')
grid on
xlabel('Time (s)')
ylabel('Rotor Speed (RPM)')
title('green actual \Omega_r, black measured \Omega_r_,_m_e_s, red reference \Omega_r_,_r_e_f')
subplot(1,2,2)
plot(time_dis,Telim,'k',time_dis,Teref,'r',time,Te,'--g',time,Tload,'m--')
grid on
xlabel('Time (s)')
ylabel('Torque (Nm)')
title('green actual T_e, red reference T_e_,_r_e_f, black limited, magenta load T_L')


