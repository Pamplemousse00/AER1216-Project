function aircraft_dynamics(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.
%
%   It should be noted that the MATLAB S-function is very similar
%   to Level-2 C-Mex S-functions. You should be able to get more
%   information for each of the block methods by referring to the
%   documentation for C-Mex S-functions.
%
%   Copyright 2003-2010 The MathWorks, Inc.

% AER1216 Fall 2025 
% Fixed Wing Project Code
%
% aircraft_dynamics.m
%
% Fixed wing simulation model file, based on the Aerosonde UAV, with code
% structure adapted from Small Unmanned Aircraft: Theory and Practice by 
% R.W. Beard and T. W. McLain. 
% 
% Inputs: 
% delta_e           elevator deflection [deg]
% delta_a           aileron deflection [deg]
% delta_r           rudder deflection [deg]
% delta_t           normalized thrust []
%
% Outputs:
% pn                inertial frame x (north) position [m]
% pe                inertial frame y (east) position [m]
% pd                inertial frame z (down) position [m]
% u                 body frame x velocity [m/s]
% v                 body frame y velocity [m/s]
% w                 body frame z velocity [m/s]
% phi               roll angle [rad]
% theta             pitch angle [rad]
% psi               yaw angle [rad]
% p                 roll rate [rad/s]
% q                 pitch rate [rad/s]
% r                 yaw rate [rad/s]
%
% Last updated: Pravin Wedage 2025-11-13


% AUTHOR NOTE: the following sections have been properly updated:
% setup
% InitializeConditions
% Output
% Derivatives

%
% The setup method is used to set up the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.
%
setup(block);

end 


%% Function: setup ===================================================
% Abstract:
%   Set up the basic characteristics of the S-function block such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
%
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
for i = 1:block.NumInputPorts
    block.InputPort(i).Dimensions        = 4;
    block.InputPort(i).DatatypeID  = 0;  % double
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).DirectFeedthrough = false; % important to be false 
end

% Override output port properties
for i = 1:block.NumOutputPorts
    block.OutputPort(i).Dimensions       = 12;
    block.OutputPort(i).DatatypeID  = 0; % double
    block.OutputPort(i).Complexity  = 'Real';
%     block.OutputPort(i).SamplingMode = 'Sample';
end

% Register parameters
block.NumDialogPrms     = 1;
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Register multiple instances allowable
% block.SupportMultipleExecInstances = true;

% Register number of continuous states
block.NumContStates = 12;

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

% -----------------------------------------------------------------
% The MATLAB S-function uses an internal registry for all
% block methods. You should register all relevant methods
% (optional and required) as illustrated below. You may choose
% any suitable name for the methods and implement these methods
% as local functions within the same file. See comments
% provided for each function for more information.
% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup); % discrete states only
block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
block.RegBlockMethod('InitializeConditions',    @InitializeConditions);
% block.RegBlockMethod('Start',                   @Start); % Initialize Conditions is used
block.RegBlockMethod('Outputs',                 @Outputs); % Required
% block.RegBlockMethod('Update',                  @Update); % only required for discrete states
block.RegBlockMethod('Derivatives',             @Derivatives); % Required for continuous states
block.RegBlockMethod('Terminate',               @Terminate); % Required

end 


%% PostPropagationSetup:
%   Functionality    : Setup work areas and state variables. Can
%                      also register run-time methods here
%   Required         : No
%   C-Mex counterpart: mdlSetWorkWidths
%
function DoPostPropSetup(block)
block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

end


%% InitializeConditions:
%   Functionality    : Called at the start of simulation and if it is 
%                      present in an enabled subsystem configured to reset 
%                      states, it will be called when the enabled subsystem
%                      restarts execution to reset the states.
%   Required         : No
%   C-MEX counterpart: mdlInitializeConditions
%
function InitializeConditions(block)

% Rename parameters
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% Initialize continuous states
block.ContStates.Data(1) = P.pn0; 
block.ContStates.Data(2) = P.pe0;
block.ContStates.Data(3) = P.pd0;
block.ContStates.Data(4) = P.u0;
block.ContStates.Data(5) = P.v0;
block.ContStates.Data(6) = P.w0;
block.ContStates.Data(7) = P.phi0;
block.ContStates.Data(8) = P.theta0;
block.ContStates.Data(9) = P.psi0;
block.ContStates.Data(10) = P.p0;
block.ContStates.Data(11) = P.q0;
block.ContStates.Data(12) = P.r0;

end 

%% Start:
%   Functionality    : Called once at start of model execution. If you
%                      have states that should be initialized once, this 
%                      is the place to do it.
%   Required         : No
%   C-MEX counterpart: mdlStart
%
function Start(block)

block.Dwork(1).Data = 0;

end 

%% Input Port Sampling Method:
function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = 'Sample';
  for i = 1:block.NumOutputPorts
    block.OutputPort(i).SamplingMode  = 'Sample';   
  end
end

%% Outputs:
%   Functionality    : Called to generate block outputs in
%                      simulation step
%   Required         : Yes
%   C-MEX counterpart: mdlOutputs
%
function Outputs(block)

temp_mat = zeros(block.NumContStates,1); % thirteen states
for i = 1:block.NumContStates
     temp_mat(i) = block.ContStates.Data(i);
end

block.OutputPort(1).Data = temp_mat; % states

% for i = 1:block.NumOutputPorts
%     block.OutputPort(1).Data(i) = block.ContStates.Data(i);
% end

end 


%% Update:
%   Functionality    : Called to update discrete states
%                      during simulation step
%   Required         : No
%   C-MEX counterpart: mdlUpdate
%
function Update(block)

block.Dwork(1).Data = block.InputPort(1).Data;

end 


%% Derivatives:
%   Functionality    : Called to update derivatives of
%                      continuous states during simulation step
%   Required         : No
%   C-MEX counterpart: mdlDerivatives
%
function Derivatives(block)

% Rename parameters
P = block.DialogPrm(1).Data; % must duplicate this line in each function

% compute inertial constants
K = P.Jx*P.Jz - (P.Jxz)^2;
k1 = P.Jxz*(P.Jx - P.Jy + P.Jz)/K ;
k2 = (P.Jz*(P.Jz - P.Jy) + (P.Jxz)^2)/K;
k3 = P.Jz/K;
k4 = P.Jxz/K;
k5 = (P.Jz - P.Jx)/P.Jy;
k6 = P.Jxz/P.Jy;
k7 = ((P.Jx - P.Jy)*P.Jx + P.Jxz^2)/K;
k8 = P.Jx/K;

% map states and inputs
pn    = block.ContStates.Data(1);
pe    = block.ContStates.Data(2);
pd    = block.ContStates.Data(3);
u     = block.ContStates.Data(4);
v     = block.ContStates.Data(5);
w     = block.ContStates.Data(6);
phi   = block.ContStates.Data(7);
theta = block.ContStates.Data(8);
psi   = block.ContStates.Data(9);
p     = block.ContStates.Data(10);
q     = block.ContStates.Data(11);
r     = block.ContStates.Data(12);
delta_e = block.InputPort(1).Data(1)*pi/180;
delta_a = block.InputPort(1).Data(2)*pi/180;
delta_r = block.InputPort(1).Data(3)*pi/180;
delta_t    = block.InputPort(1).Data(4);

% Air Data 
Va = sqrt(u^2 + v^2 + w^2);
alpha = atan2(w,u);
beta = asin(v/Va);

% rotation matrix
R_roll = [...
      1, 0, 0;...
      0, cos(phi), sin(phi);...
      0, -sin(phi), cos(phi)];
R_pitch = [...
      cos(theta), 0, -sin(theta);...
      0, 1, 0;...
      sin(theta), 0, cos(theta)];
R_yaw = [...
      cos(psi), sin(psi), 0;...
      -sin(psi), cos(psi), 0;...
      0, 0, 1];
R_T = R_roll*R_pitch*R_yaw ;   
R = R_T';

% Aerodynamic Coefficients
C_L_a = P.C_L_0 + P.C_L_alpha*alpha;
C_D_a = P.C_D_0 + P.C_D_alpha*alpha;

C_X_a = -C_D_a*cos(alpha) + C_L_a*sin(alpha);
C_X_q = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
C_X_de = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
C_X = (C_X_a + C_X_q*P.c*q/(2*Va) + C_X_de*delta_e);

C_Z_a = -C_D_a*sin(alpha) - C_L_a*cos(alpha);
C_Z_q = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
C_Z_de = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
C_Z = C_Z_a + C_Z_q*P.c*q/(2*Va) + C_Z_de*delta_e;

C_m = P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c*q/(2*Va) ...
     + P.C_m_delta_e*delta_e;

C_Y = P.C_Y_0 + P.C_Y_beta*beta...
      + P.C_Y_p*P.b*p/(2*Va)...
      + P.C_Y_r*P.b*r/(2*Va)...
      + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;
  
C_l = P.C_l_0 + P.C_l_beta*beta + P.C_l_p*P.b*p/(2*Va)... 
     + P.C_l_r*P.b*r/(2*Va) + P.C_l_delta_a*delta_a + P.C_l_delta_r*delta_r;
 
C_n = P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b*p/(2*Va) ...
    + P.C_n_r*P.b*r/(2*Va) + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r;

% aerodynamic forces and moments
fx_a = 0.5*P.rho*Va^2*P.S*C_X;
fy_a = 0.5*P.rho*Va^2*P.S*C_Y;
fz_a = 0.5*P.rho*Va^2*P.S*C_Z;
tx_a = 0.5*P.rho*Va^2*P.S*P.b*C_l;
ty_a = 0.5*P.rho*Va^2*P.S*P.c*C_m;
tz_a = 0.5*P.rho*Va^2*P.S*P.b*C_n;

% propulsion forces and moments

% Convert throttle to shaft speed (simple linear map)
n = delta_t * P.rev_max;   % [rev/s]

% Avoid divide-by-zero if throttle is zero
if n < 1e-3
    fx_p = 0;
    fy_p = 0;
    fz_p = 0;
    tx_p = 0;
    ty_p = 0;
    tz_p = 0;
else
    % Advance ratio
    J = Va / (n * P.D_prop);

    % Interpolate coefficients from table
    CT = interp1(P.J_ref, P.CT_ref, J, 'linear', 'extrap');
    CP = interp1(P.J_ref, P.CP_ref, J, 'linear', 'extrap');

    % Thrust and shaft power
    T = P.rho * n^2 * P.D_prop^4 * CT;        % [N]
    P_shaft = P.rho * n^3 * P.D_prop^5 * CP;  % [W]

    % Map to body forces/moments
    fx_p = T;   % thrust along +x body
    fy_p = 0;
    fz_p = 0;

    tx_p = 0;
    ty_p = 0;
    tz_p = 0;
end


% gravity
fx_g = -P.mass*P.g*sin(theta);
fy_g = P.mass*P.g*cos(theta)*sin(phi);
fz_g = P.mass*P.g*cos(theta)*cos(phi);

% total forces and moments (body frame)
fx = fx_a + fx_p + fx_g;
fy = fy_a + fy_p + fy_g;
fz = fz_a + fz_p + fz_g;
tx = tx_a + tx_p;
ty = ty_a + ty_p;
tz = tz_a + tz_p;

% state derivatives
pdot = R*[u, v, w]';
pndot = pdot(1);
pedot = pdot(2);
pddot = pdot(3);

udot = r*v - q*w + fx/P.mass;
vdot = p*w - r*u + fy/P.mass;
wdot = q*u - p*v + fz/P.mass;

phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
thetadot = cos(phi)*q - sin(phi)*r ;
psidot = sin(phi)*q/cos(theta) + cos(phi)*r/cos(theta);

pdot = k1*p*q - k2*q*r + k3*tx + k4*tz;
qdot = k5*p*r - k6*(p^2 - r^2) + ty/P.Jy;
rdot = k7*p*q - k1*q*r + k4*tx + k8*tz;

% map derivatives
block.Derivatives.Data(1) = pndot;
block.Derivatives.Data(2) = pedot;
block.Derivatives.Data(3) = pddot;
block.Derivatives.Data(4) = udot;
block.Derivatives.Data(5) = vdot;
block.Derivatives.Data(6) = wdot;
block.Derivatives.Data(7) = phidot;
block.Derivatives.Data(8) = thetadot;
block.Derivatives.Data(9) = psidot;
block.Derivatives.Data(10)= pdot;
block.Derivatives.Data(11)= qdot;
block.Derivatives.Data(12)= rdot;

end 


%% Terminate:
%   Functionality    : Called at the end of simulation for cleanup
%   Required         : Yes
%   C-MEX counterpart: mdlTerminate
%
function Terminate(block)

end 

