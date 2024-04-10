%**********************************************************************
%   Template - Electric Drive systems ENM076
%**********************************************************************
%
clc         % clear command window
clear all   % clear workspace memory
close all   % closing all plot windows

%*******************************************************************
% Parameter definition (4 kW 4-pole machine)
Rs  =1.33;              % Stator resistance
Rr  =1.24;              % Rotor resistance      
Lm  =0.135;             % Magnetizing inductance
Lsl =0.008;             % Stator leakage inductance
Lrl =0.008;             % Rotor leakage inductance
Ls  =Lm+Lsl;            % Stator inductance
Lr  =Lm+Lrl;            % Rotor inductance
np  = 2;                % Pole pair number
J   =0.05*0.25;         % Inertia of IM
B   =0.08;              % Mechanical damping
% Invers Gamma param
RR     = (Lm/Lr)^2*Rr;
LM     = Lm^2/Lr;
Lsigma = Ls-LM;

%***********************************************
% Battery and converter
Vbatt = 625;            % Battery voltage [V]
Vdc0 = Vbatt;           % Initial condition for the capacitor voltage in the converter [V]
Rbatt = 1;              % Internal resistance of the battery [ohm]
Cdc = 5*470e-6;         % DC capacitance of the converter [F]

%***************************************************************************************************************************************************************
% Control system
Ts = 1/5000;  % Sampling period
%Ts = 0.33e-3; % aproximately = 1/3000 Sampling period
%Ts = 1/2000;  % Sampling period

% Define all the required parameteres for the controller here
% rated current = 9.1*sqrt(2)*1.2 A
% current control BW = 1000 rad/s
% speed control BW = 20 rad/s

% Limits
Israted = 9.1*sqrt(2)*1.2;      % Current limiter

% Parameter estimation
Rshat     = Rs;
RRhat     = RR;
Lsigmahat = Lsigma;
%Lsigmahat = 0.5*Lsigma;
%Lsigmahat = 2*Lsigma;
LMhat     = LM;
Jhat      = J;
Bhat      = B;

% Current controller
alphac = 1000;
kpc = alphac * Lsigmahat;
Ra = Lsigmahat * alphac - Rshat - RRhat;
kic = alphac * (RRhat + Rshat + Ra); 

% Speed controller
alphas = 20;
kps = alphas * Jhat; 
Ba = alphas * Jhat - Bhat;
kis = alphas * (Bhat + Ba);





%********************************************
% Reference signals

% Rotor flux, invers gamma
Flux_ref = 0.66*sqrt(2)*Lm/Lr;
Flux_ref_time = 0.001;

% Extra load torque
TL_extra_time=1;
TL_extra=14.4;

% Speed reference
speed_ref_time = 0.5;
speed_ref  = 1435*pi/30;

% Torque reference for Ass7.e)
Te_ref_time = 0.5;
Te_ref_value = 12;

%*************************************
% Simulation,                    
% Initiating the simulation
% xi= initial conditions
% state variables=isalpha,isbeta,iralpha,irbeta,wr,theta_r
xi=[0;0;0;0;0;0];	% No currents, no rotation, postion=0          
% Call solver using panel settings, i.e Variable-step
%**********************************************************************
Tstart=0;
Tstop=2.5;
%Tstop=0.55;
sim('IMpanel',[Tstart,Tstop])
%************************************************
%  postprocessning part, Do not change this part*
%************************************************
PostPross

