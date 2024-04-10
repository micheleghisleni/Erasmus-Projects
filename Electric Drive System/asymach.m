function [sys,x0,str,ts] = asymach(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi)
%CSFUNC An example M-file S-function for defining a continuous system.  
%   Example M-file S-function implementing continuous equations: 
%      x' = Ax + Bu
%      y  = Cx + Du
%   See sfuntmpl.m for a general S-function template.

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi);
    
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
% end csfunc
%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi)

sizes = simsizes;
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = xi;
str = [];
ts  = [0 0];
% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi)

%state variables:
isalpha=x(1);
isbeta=x(2);
iralpha=x(3);
irbeta=x(4);
wr=x(5);
theta=x(6);

% input signals:
Usalpha=u(1);	% Stator voltage alpha
Usbeta=u(2);	% Stator voltage beta
TL_extra=u(3);	% Shaft torque

%defining stator and rotor inductance
Ls=Lsl+Lm;
Lr=Lrl+Lm;

% Defining the resistance matrix RMAT
RMAT=[Rs,0,0,0;
	0,Rs,0,0;
	0,wr*Lm,Rr,wr*Lr;
	-wr*Lm,0,-wr*Lr,Rr];

% Defining the inductance matrix LMAT
LMAT=[Ls,0,Lm,0;
	0,Ls,0,Lm;
	Lm,0,Lr,0;
	0,Lm,0,Lr];

% forming the state-space matrices
BMAT=inv(LMAT);
AMAT=-inv(LMAT)*RMAT;

% formation of the 4 current derivatives xdot=Ax+Bu  (sys=xdot)
sys(1:4)=AMAT*x(1:4)+BMAT*[Usalpha;Usbeta;0;0];

% forming the electrodynamical torque
Te=3*np/2*Lm*(iralpha*isbeta-isalpha*irbeta);

% forming the load torque
Tload=B*wr/np+TL_extra;  

%  forming the rotor speed derivative d/dt(wr)=............ 
%  sys(5)=d/dt(wr)
sys(5)=np/J*(Te-Tload);

% forming of the angle theta d/dt(theta)=wr
sys(6)=wr;

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,flag,Rs,Rr,Lm,Lsl,Lrl,J,np,B,xi)

Ls=Lm+Lsl;
Lr=Lm+Lrl;

isalpha=x(1);
isbeta=x(2);
iralpha=x(3);
irbeta=x(4);
wr=x(5);
theta=x(6);

% determination of rotor flux in alpha and beta direction
%
Psiralpha=Lr*iralpha+Lm*isalpha;
Psirbeta=Lr*irbeta+Lm*isbeta;

% determination of rotor flux angle (use 'atan2')
Psir_angle=atan2(Psirbeta,Psiralpha);

% determination of rotor flux magnitude  (use 'abs')
Psir=sqrt(Psiralpha^2+Psirbeta^2);

% determination of electromagnetic torque
Te=3*np/2*Lm*(iralpha*isbeta-isalpha*irbeta);

sys=[isalpha;isbeta;iralpha;irbeta;Te;wr/np;theta/np;Psir;Psir_angle];
% end mdlOutputs