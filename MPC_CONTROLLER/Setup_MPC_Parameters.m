function mpc_params = Setup_MPC_Parameters()
% Setup_MPC_Parameters - Centralised configuration for the MPC Controller
% 
% This function defines all parameters required for operation
% of the Model Predictive Controller implemented in the function mpcController().
% 
% The separation of configuration and control logic makes it possible to:
% - Easily change parameters without touching the controller code
% - Maintain a consistent configuration throughout the project
% - Simplify tuning of the controller
% - Allow different configurations for different scenarios
%
% Output:
% mpc_params: structure containing all MPC controller parameters
%

%% =====================================================================
%% VEHICLE PARAMETERS
%% =====================================================================
% These parameters describe the physical characteristics of the vehicle

% Wheelbase del veicolo [m] - distance between front and rear axle
mpc_params.wheelbase = 2.8;

% Physical constraints of the vehicle
mpc_params.v_max = 2;        
mpc_params.v_min = 0;       
mpc_params.phi_max = pi/6;    
mpc_params.a_max = 1.5;        
mpc_params.a_min = -3.0;      

fprintf('  - Wheelbase: %.1f m\n', mpc_params.wheelbase);
fprintf('  - Velocity: %.1f - %.1f m/s\n', mpc_params.v_min, mpc_params.v_max);
fprintf('  - Max steer Angle: %.1f degrees\n', mpc_params.phi_max * 180/pi);

%% =====================================================================
%% TIME PARAMETERS AND PREDICTION HORIZON
%% =====================================================================
% These parameters define the 'time window' over which the MPC
% optimises control decisions

% Sampling Time [s] 
mpc_params.sampling_time = 0.1;

% Prediction horizon - number of future steps considered
mpc_params.prediction_horizon = 6;

% Control horizon - number of optimised control variables
mpc_params.control_horizon = 4;
