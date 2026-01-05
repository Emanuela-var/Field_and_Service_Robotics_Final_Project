% =======================================================================
% === AUTOMATIC TUNING SCRIPT FOR MPC PARAMETERS =====================
% =======================================================================
%
% This script iteratively runs the Simulink simulation with different
% combinations of MPC parameters to find the optimal configuration.
%

clear;
clc;
close all;

fprintf('=== MPC TUNING PROCESS STARTED ===\n');

%% 1. TUNING CONFIGURATION

simulink_model_name = 'MPC_CONTROLLER'; % 
load('desired_trajectory.mat');

% Values for the prediction horizon (Np)
Np_vals = [12, 15, 20];

% Values for the weight of the lateral error (Q(1))
Q_lateral_vals = [150, 250, 400, 600];

% Values for the weight of the steering cost (R(2))
R_steer_vals = [15, 8, 4, 1];

Q_longitudinal = 80;
Q_heading = 70;
R_velocity = 1;


%% 2. EXECUTION OF SIMULATIONS

total_sims = length(Np_vals) * length(Q_lateral_vals) * length(R_steer_vals);
fprintf('Total number of configurations to be tested: %d\n', total_sims);

results = table();
sim_count = 0;

% Tuning
for Np = Np_vals
    for Q_lat = Q_lateral_vals
        for R_steer = R_steer_vals
            
            sim_count = sim_count + 1;
            fprintf('\n--- Execution Simulation %d/%d ---\n', sim_count, total_sims);
            
            current_params = mpc_params;
            current_params.prediction_horizon = Np;
            current_params.Q_weights = [Q_lat, Q_longitudinal, Q_heading];
            current_params.R_weights = [R_velocity, R_steer];
            
            fprintf('Current parameters: Np=%d, Q_lat=%.1f, R_steer=%.1f\n', Np, Q_lat, R_steer);
            
            
            try
                
                simOut = sim(simulink_model_name, 'ReturnWorkspaceOutputs', 'on', ...
                                                  'LoadInitialState', 'off');

               
                out_actual_state = simOut.out_actual_state;

              
                time_actual = out_actual_state.Time;
                state_actual = squeeze(out_actual_state.Data); 
                if size(state_actual,1) ~= length(time_actual), state_actual = state_actual'; end
                
                
                t_start = max(time_vector(1), time_actual(1));
                t_end = min(time_vector(end), time_actual(end));
                time_eval = t_start:0.1:t_end;

                xd_interp = interp1(time_vector, trajectory.xd, time_eval);
                yd_interp = interp1(time_vector, trajectory.yd, time_eval);
                
                x_interp = interp1(time_actual, state_actual(:,1), time_eval);
                y_interp = interp1(time_actual, state_actual(:,2), time_eval);
                
                
                error_position = sqrt((x_interp - xd_interp).^2 + (y_interp - yd_interp).^2);
                
                
                mean_pos_err = mean(error_position, 'omitnan');
                max_pos_err = max(error_position, [], 'omitnan');
                final_pos_err = error_position(end); 
                
                fprintf('Results: Error Pos. Medium=%.4f m, Max=%.4f m, Final=%.4f m\n', ...
                        mean_pos_err, max_pos_err, final_pos_err);

               
                new_row = {Np, Q_lat, R_steer, mean_pos_err, max_pos_err, final_pos_err};
                results = [results; new_row];
                
            catch ME
                fprintf('!!! ERROR during simulation for Np=%d, Q_lat=%.1f, R_steer=%.1f !!!\n', Np, Q_lat, R_steer);
                fprintf('Error message: %s\n', ME.message);
                
                
                new_row = {Np, Q_lat, R_steer, NaN, NaN, NaN};
                results = [results; new_row];
            end
        end
    end
end

%% 3. ANALYSIS OF RESULTS

fprintf('\n\n=== TUNING COMPLETED ===\n');

results.Properties.VariableNames = {'Np', 'Q_lateral', 'R_steer', 'MeanPosError', 'MaxPosError', 'FinalPosError'};

results = rmmissing(results);

if isempty(results)
    error('No simulation successfully completed. Check Simulink model.');
end

results_sorted = sortrows(results, 'MeanPosError', 'ascend');

fprintf('--- The 10 Best Configurations (sorted by Mean Error) ---\n');
disp(results_sorted(1:min(10, height(results_sorted)), :));

best_params_row = results_sorted(1, :);
best_Np = best_params_row.Np;
best_Q_lat = best_params_row.Q_lateral;
best_R_steer = best_params_row.R_steer;

fprintf('\n=== OPTIMAL CONFIGURATION FOUND ===\n');
fprintf('Prediction Horizon (Np): %d\n', best_Np);
fprintf('Lateral Error Weight (Q_lat): %.1f\n', best_Q_lat);
fprintf('Weight Steering Cost (R_steer): %.1f\n', best_R_steer);
fprintf('\nWith the following performances:\n');
fprintf('-> Mean Positional Error: %.4f m\n', best_params_row.MeanPosError);
fprintf('-> Maximum Positional Error: %.4f m\n', best_params_row.MaxPosError);
fprintf('-> Final Positional Error: %.4f m\n', best_params_row.FinalPosError);

%% 4. VISUALISATION 
figure('Name', 'MPC Tuning Analysis');
parallelplot(results_sorted, 'CoordinateVariables', {'Np', 'Q_lateral', 'R_steer'}, ...
                             'GroupVariable', 'MeanPosError', 'ColorMap', 'jet');
title('Relationship between Parameters and Mean Tracking Error');
xlabel('Tuning Parameters');
ylabel('Value Parameter');

fprintf('\nProcess terminated. Update MAIN_MPC.m file with optimal parameters.\n');