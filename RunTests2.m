% Define your test seeds
test_seeds = [111, 222, 333];

% Name of your Simulink model file
mdl = 'AUVdepthHeadingControlWithRealisticWind';
load_system(mdl);

% =========================================================
% DEFINE THE TEST SUITE (Tiers 1, 2, and 3)
% =========================================================
% Assumption for WindEnum: 1=Calm, 2=Normal, 3=Stormy

% Test 1: The Broadside Wall (Stormy, 5m Hold, Max Whiplash)
tests(1) = struct('name', 'The Broadside Wall', 'wind', 3, 'z0', 5, 'zf', 5, 'h0', 0, 'hf', 0);

% Test 2: The Ekman Corkscrew (Normal, 1m to 50m Descent, Hold Heading)
tests(2) = struct('name', 'The Ekman Corkscrew', 'wind', 2, 'z0', 1, 'zf', 50, 'h0', 0, 'hf', 0);

% Test 3: The Silent Windup (Calm, 30m Hold)
tests(3) = struct('name', 'The Silent Windup', 'wind', 1, 'z0', 30, 'zf', 30, 'h0', 0, 'hf', 0);

% Test 4: The Surface Breach (Stormy, 100m to 5m Ascent)
tests(4) = struct('name', 'The Surface Breach', 'wind', 3, 'z0', 100, 'zf', 5, 'h0', 0, 'hf', 0);

% Test 5: The Operational Envelope (Normal, 10m Hold, Maneuver 0 to -60)
tests(5) = struct('name', 'The Operational Envelope', 'wind', 2, 'z0', 10, 'zf', 10, 'h0', 0, 'hf', -60);

% Test 6: The Deep Freeze (Stormy, 100m Hold, Maneuver 0 to -60)
tests(6) = struct('name', 'The Deep Freeze', 'wind', 3, 'z0', 100, 'zf', 100, 'h0', 0, 'hf', -60);

% =========================================================
% EXECUTE BATCH SIMULATIONS
% =========================================================

% Loop through each of the 6 tests
for t = 1:length(tests)
    fprintf('==================================================\n');
    fprintf('STARTING TEST: %s\n', tests(t).name);
    fprintf('==================================================\n');
    
    % Loop through each seed for the current test
    for i = 1:length(test_seeds)
        fprintf('  Running Seed %d ... ', test_seeds(i));
        
        % 1. Create the SimulationInput object
        dummy = Simulink.SimulationInput(mdl);
        
        % 2. Pass the specific test parameters into the simulation sandbox
        dummy = setVariable(dummy, 'active_seed', test_seeds(i));
        dummy = setVariable(dummy, 'WindConditionsEnum', tests(t).wind);
        dummy = setVariable(dummy, 'Initial_depth', tests(t).z0);
        dummy = setVariable(dummy, 'Final_depth', tests(t).zf);
        dummy = setVariable(dummy, 'Initial_heading', tests(t).h0);
        dummy = setVariable(dummy, 'Final_heading', tests(t).hf);
        
        % 3. Run the simulation
        dummy_out = sim(dummy);
        
        % 4. Extract standard data 
        direction_signal = dummy_out.yout.get('Pre_Wrap_Current_Direction');
        direction_data = direction_signal.Values.Data;
        time_data = direction_signal.Values.Time; 
        
        % 5. Test-Specific Analytics
        if t == 1 
            % --- Broadside Wall Logic (Find t_worst) ---
            direction_change = diff(direction_data);
            l = length(direction_change);
            
            offset = round(l / 10);
            start_idx = max(1, offset);
            end_idx = l - offset;
            
            sliced_direction_change = direction_change(start_idx:end_idx);
            [~, local_worst_index] = max(abs(sliced_direction_change));
            global_worst_index = start_idx + local_worst_index - 1;
            t_worst = time_data(global_worst_index + 1); 
            
            fprintf('Done. Most violent shift at t = %.1f s\n', t_worst);
            
        elseif t == 2 || t == 4
            % --- Ascent/Descent Settling Time Logic ---
            % Extract the depth data from logsout
            depth_signal = dummy_out.logsout.get('z');
            depth_data = depth_signal.Values.Data;
            
            % If your scope logs both Reference and Actual on the same wire,
            % it creates a 2D matrix. We only want the actual physical depth.
            if size(depth_data, 2) > 1
                depth_data = depth_data(:, 1); % Grabs the first column (usually the actual state)
            end
            
            target_depth = tests(t).zf;
            start_depth = tests(t).z0;
            
            % Define the tolerance band (e.g., 2% of the total distance traveled)
            % For a 95m ascent, this means it is "complete" when it stays within 1.9m of the target.
            pct_tol = 0.02;
            tolerance = pct_tol * abs(target_depth - start_depth); 
            
            % Find every single timestamp where the AUV is OUTSIDE the tolerance band
            error_array = abs(depth_data - target_depth);
            outside_tol_indices = find(error_array > tolerance);
            
            % Calculate Settling Time
            if isempty(outside_tol_indices)
                % It was already at the target depth
                t_complete = 0;
                fprintf('Done. Ascent/Descent complete at t = 0.0 s\n');
                
            elseif outside_tol_indices(end) == length(depth_data)
                % If the very last index of the simulation is STILL outside the tolerance,
                % the controller failed to reach the target before the 10 hours ended.
                t_complete = NaN; 
                fprintf('Done. WARNING: Failed to settle within %d%% of target.\n', round(pct_tol*100));
                
            else
                % The completion time is the exact time step immediately after it 
                % exits the tolerance band for the very last time.
                settling_index = outside_tol_indices(end) + 1;
                t_complete = time_data(settling_index);
                fprintf('Done. Ascent/Descent complete at t = %.1f s\n', t_complete);
            end

        else
            % --- Placeholder for other test analytics ---
            % (e.g., Test 5 & 6 complex maneuver tracking error)
            fprintf('Done.\n');
        end
        
        % Optional: If you want to automatically save the dataset for later plotting
        filename = sprintf('%s_Seed_%d.mat', strrep(tests(t).name, ' ', '_'), test_seeds(i));
        save(filename, 'dummy_out');
        
    end
    fprintf('\n'); % Spacing between test tiers
end

disp('All Test Tiers Complete!');