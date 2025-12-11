% AER1216 Fall 2025 
% Fixed Wing Project Code
%
% parameters.m
%
% Initialization file which generates and stores all required data into the 
% structure P, which is then stored in the workspace. Simulink model calls 
% on this function at the start of every simulation. Code structure adapted
% from Small Unmanned Aircraft: Theory and Practice by R.W. Beard and T. W. 
% McLain. 
% 
% Inputs: 
% N/A
%
% Outputs:
% P                 structure that contains all aerodynamic, geometric, and
%                   initial condition data for the aircraft and simulation.
%
% Last updated: Pravin Wedage 2025-11-13


% Initial Conditions
clear all
% compute trim conditions            
P.Va0 = 2;         % initial airspeed (also used as trim airspeed)
P.Va_trim = 25; 
P.Va = P.Va_trim;
gamma = 0*pi/180;   % desired flight path angle (radians)
    R = inf;        % desired radius (m) 
                    % use (+) for right handed orbit, (-) for left handed orbit 


P.gravity = 9.81;
P.g = 9.81; 

% Fixed Wing
% physical parameters of airframe
%from XFLR5 output - stability analysis
P.mass = 6.2775;   % [kg]
P.Jx   = 0.2622;   % [kg m^2] Ibxx
P.Jy   = 0.5319;   % [kg m^2] Ibyy
P.Jz   = 0.7549;   % [kg m^2] Ibzz
P.Jxz  = 0.03245;  % [kg m^2] Ibxz

%kept from Pravins version
gam0 = P.Jx*P.Jz - (P.Jxz)^2;
gam1 = P.Jxz*(P.Jx - P.Jy + P.Jz)/gam0 ;
gam2 = (P.Jz*(P.Jz - P.Jy) + (P.Jxz)^2)/gam0;
gam3 = P.Jz/gam0;
gam4 = P.Jxz/gam0;
gam5 = (P.Jz - P.Jx)/P.Jy;
gam6 = P.Jxz/P.Jy;
gam7 = ((P.Jx - P.Jy)*P.Jx + P.Jxz^2)/gam0;
gam8 = P.Jx/gam0;
P.gam0 = gam0;
P.gam1 = gam1;
P.gam2 = gam2;
P.gam3 = gam3;
P.gam4 = gam4;
P.gam5 = gam5;
P.gam6 = gam6;
P.gam7 = gam7;
P.gam8 = gam8;

%from XFLR5 plane design
P.S_wing        = 0.704;  %  [m^2]
P.S             = 0.704;  %  [m^2]
P.b             = 2.420;  %  Wingspan [m]
P.c             = 0.291;  %  MAC [m]

% aerodynamic coefficients
P.S_prop        = 0.2027; %neeraj input
P.rho           = 1.21366; % rho @ cruise (100m) [kg/m^3]
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

%longtitudinal aerodynamics
P.C_L_0         = 0.45;    %est trim lift
P.C_L_alpha     = 5.2852;  %CLa
P.C_L_q         = 8.1233;  %CLq
P.C_L_delta_e   = 0.79628;    %CZde (elevator lift - check sign)

P.C_D_0         = 0.025;   %Parasitic drag build up
P.C_D_alpha     = 0.05;    %Induced drag
P.C_D_p         = 0.0;     %ignore rn
P.C_D_q         = 0.0;     %ignore rn
P.C_D_delta_e   = 0.025597;    %CXde

P.C_m_0         = 0;       %Trimmed condition
P.C_m_alpha     = -0.70099; %Cma (pitch stability)
P.C_m_q         = -23.185; %Cmq (pitch damping)
P.C_m_delta_e   = -2.8604;   %CMde (control power)

%lateral aerodynamics
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.4175; %CYb
P.C_Y_p         = -0.0817; %CYp
P.C_Y_r         = 0.4191;  %CYr
P.C_Y_delta_a   = 0.0138;  %CYde Aileron
P.C_Y_delta_r   = 0.2434;  %CYde Rudder

P.C_l_0       = 0.0;
P.C_l_beta    = -0.0773;   %Clb
P.C_l_p       = -0.5527;   %Clp
P.C_l_r       = 0.1895;    %Clr
P.C_l_delta_a = 0.5234;    %CLde Aileron
P.C_l_delta_r = 0.0231;    %CLde Rudder

P.C_n_0         = 0.0;
P.C_n_beta      = 0.1841;  %Cnb
P.C_n_p         = -0.0705; %Cnp
P.C_n_r         = -0.1842; %Cnr
P.C_n_delta_a   = 0.0208;  %CNde Aileron
P.C_n_delta_r   = -0.1151; %CNde Rudder

P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

P.C_p_0         = gam3*P.C_l_0 + gam4*P.C_n_0;
P.C_p_beta      = gam3*P.C_l_beta + gam4*P.C_n_beta;
P.C_p_p         = gam3*P.C_l_p + gam4*P.C_n_p;
P.C_p_r         = gam3*P.C_l_r + gam4*P.C_n_r;
P.C_p_delta_a   = gam3*P.C_l_delta_a + gam4*P.C_n_delta_a;
P.C_p_delta_r   = gam3*P.C_l_delta_r + gam4*P.C_n_delta_r;
P.C_r_0         = gam4*P.C_l_0 + gam8*P.C_n_0;
P.C_r_beta      = gam4*P.C_l_beta + gam8*P.C_n_beta;
P.C_r_p         = gam4*P.C_l_p + gam8*P.C_n_p;
P.C_r_r         = gam4*P.C_l_r + gam8*P.C_n_r;
P.C_r_delta_a   = gam4*P.C_l_delta_a + gam8*P.C_n_delta_a;
P.C_r_delta_r   = gam4*P.C_l_delta_r + gam8*P.C_n_delta_r;

% Control Input limits 
P.delta_e_max = deg2rad(45); % assumed symmetric
P.delta_a_max = deg2rad(45); 
P.delta_r_max = deg2rad(25);
P.phi_max = deg2rad(30);        % roll error max
P.beta_max = deg2rad(10);       % sideslip error max (not in rads, care)
P.theta_max = deg2rad(2);      % pitch error max
% P.theta_max = deg2rad(10);      % pitch error max

% % Thrust parameters
% prop_data = importdata('ancf_16x8_1001od_3967.txt');
%         P.J_ref = prop_data.data(:,1);
%         P.CT_ref = prop_data.data(:,2);
%         P.CP_ref = prop_data.data(:,3);
%         P.eta_ref = prop_data.data(:,4);
%         P.rev_max = 5000/60;
%         P.S_prop = 0.2027;                      % [m^2]
%         P.D_prop = sqrt(4*P.S_prop/pi);         % [m]
prop_data = importdata('propData_8k.txt');
        P.J_ref = prop_data.data(:,1);
        P.CT_ref = prop_data.data(:,2);
        P.CP_ref = prop_data.data(:,3);
        P.eta_ref = prop_data.data(:,4);
        P.rev_max = 10656/60;
        P.S_prop = 0.2027;                      % [m^2]
        P.D_prop = sqrt(4*P.S_prop/pi);         % [m]
 

% wind parameters
P.wind_n = 0; %3;
P.wind_e = 0; %2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = 0.7;
P.wind_gain = 2;
P.wind_gain_n = 2;
P.wind_gain_e = 2;
P.wind_gain_d = 2;

% Initial Conditions
pn       = 0;  % initial North position
pe       = 0;  % initial East position
pd       = -1.5;  % initial Down position (negative altitude)
P.pn0    = pn;  % initial North position
P.pe0    = pe;  % initial East position
P.pd0    = pd;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axisu_trim
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
psi      = 0;  % initial yaw angle
P.psi0   = psi;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
P.delta_e0 =0;
P.delta_a0 =0;
P.delta_r0 =0;
P.delta_t0 =1;




global K_px K_dx K_py K_dy K_pz K_dz K_p_phi K_d_phi K_p_theta K_d_theta K_p_psi K_d_psi;

K_p_yaw = 1;
K_i_yaw = 1;
K_d_yaw = 1;

K_p_roll = 1;
K_i_roll = 1;
K_d_roll = 1;

K_p_climb_rate = 1;
K_i_climb_rate = 1;
K_d_climb_rate = 1;

K_p_altitude = 1;
K_i_altitude = 1;
K_d_altitude = 1;

K_p_pitch = 1;
K_i_pitch = 1;
K_d_pitch = 1;

K_p_velocity = 1;
K_i_velocity = 1;
K_d_velocity = 1;

K_p_sideslip = 1;
K_i_sideslip = 1;
K_d_sideslip = 1;

climb_theta_gain = 1;
climb_V_gain = 0;


V_c_ts = timeseries;
h_c_ts = timeseries;
hdot_c_ts = timeseries;
psi_c_ts = timeseries;

mission_commands;

