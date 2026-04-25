% =========================================================
% RunTests2.m
% Batch Test Runner — BlueROV2 PID Depth & Heading Controller
% Chris Hunt, Jack Kamataris — RBE502 ROV Control Project
%
% Run ROV_PID_Init.m first to load controller gains into workspace.
% =========================================================
clear workspace
% Define your test seeds
test_seeds = [111, 222, 333];
% test_seeds = [111];
tests_to_run = [1:6];
% Name of your Simulink model file
mdl = 'AUVdepthHeadingControlWithRealisticWind';
load_system(mdl);
clear tests;
% =========================================================
% DEFINE THE TEST SUITE (Tiers 1, 2, and 3)
% =========================================================
% WindEnum: 1=Calm, 2=Normal, 3=Stormy
kp=0.08;
ki=0;
kd=20;


% Test 1: The Broadside Wall (Stormy, 5m Hold, Max Whiplash)
tests(1) = struct('name', 'The Broadside Wall',       'wind', 3, 'z0', 5,   'zf', 5,   'h0', 0, 'hf', 0,   'Kpz', kp,  'Kiz',ki,   'Kdz',kd);

% Test 2: The Ekman Corkscrew (Stormy, 1m to 50m Descent, Hold Heading)
tests(2) = struct('name', 'The Ekman Corkscrew',      'wind', 3, 'z0', 1,   'zf', 50,  'h0', 0, 'hf', 0,   'Kpz', kp,  'Kiz',ki,   'Kdz',kd);

% Test 3: The Silent Windup (Calm, 30m Hold)
tests(3) = struct('name', 'The Silent Windup',        'wind', 1, 'z0', 30,  'zf', 30,  'h0', 0, 'hf', 0,   'Kpz', kp,  'Kiz',ki,   'Kdz',kd);

% Test 4: The Surface Breach (Stormy, 100m to 5m Ascent)
tests(4) = struct('name', 'The Surface Breach',       'wind', 3, 'z0', 100, 'zf', 5,   'h0', 0, 'hf', 0,   'Kpz', kp,  'Kiz',ki,   'Kdz',kd);

% Test 5: The Operational Envelope (Normal, 10m Hold, Maneuver 0 to -60 deg)
tests(5) = struct('name', 'The Operational Envelope', 'wind', 2, 'z0', 10,  'zf', 10,  'h0', 0, 'hf', -60, 'Kpz', kp,  'Kiz',ki,   'Kdz',kd);

% Test 6: The Deep Freeze (Stormy, 100m Hold, Maneuver 0 to -60 deg)
tests(6) = struct('name', 'The Deep Freeze',          'wind', 3, 'z0', 100, 'zf', 100, 'h0', 0, 'hf', -60, 'Kpz', kp,  'Kiz',ki,   'Kdz',kd);

% =========================================================
% EXECUTE BATCH SIMULATIONS
% =========================================================
for t = tests_to_run
    fprintf('==================================================\n');
    fprintf('STARTING TEST: %s\n', tests(t).name);
    fprintf('==================================================\n');

    for i = 1:length(test_seeds)
        fprintf('  Running Seed %d ... ', test_seeds(i));

        % Initialize a clean metrics struct
        test_metrics = struct();
        test_metrics.test_name = tests(t).name;
        test_metrics.seed      = test_seeds(i);

        % Create the SimulationInput object
        dummy = Simulink.SimulationInput(mdl);

        % Pass test parameters
        dummy = setVariable(dummy, 'active_seed',         test_seeds(i));
        dummy = setVariable(dummy, 'WindConditionsEnum',  tests(t).wind);
        dummy = setVariable(dummy, 'Initial_depth',       tests(t).z0);
        dummy = setVariable(dummy, 'Final_depth',         tests(t).zf);
        dummy = setVariable(dummy, 'Initial_heading',     tests(t).h0);
        dummy = setVariable(dummy, 'Final_heading',       tests(t).hf);
        dummy = setVariable(dummy, 'Kpz', tests(t).Kpz);
        dummy = setVariable(dummy, 'Kiz', tests(t).Kiz);
        dummy = setVariable(dummy, 'Kdz', tests(t).Kdz);
        blk = find_system(mdl, 'BlockType', 'Step');
        depth_blk = blk{1};  % Desired depth block
        dummy = setBlockParameter(dummy, depth_blk, 'Before', num2str(tests(t).z0));
         
        % Run the simulation
        dummy_out = sim(dummy);
        % try
        %     disp(dummy_out.logsout.getElementNames());
        % catch
        %     disp('logsout not present');
        % end
        % Extract current direction signal (available in all tests)
        direction_signal = dummy_out.yout.get('Pre_Wrap_Current_Direction');
        direction_data   = direction_signal.Values.Data;
        time_data        = direction_signal.Values.Time;

        % =========================================================
        % TEST-SPECIFIC ANALYTICS
        % =========================================================

        if t == 1
            % --- Test 1: Broadside Wall ---
            % Find the moment of most violent current direction shift.
            direction_change = diff(direction_data);
            l = length(direction_change);

            offset    = round(l / 4);
            start_idx = max(1, offset);
            end_idx   = l - round(l/10);

            sliced_direction_change = direction_change(start_idx:end_idx);
            [~, local_worst_index]  = max(abs(sliced_direction_change));
            global_worst_index      = start_idx + local_worst_index - 1;
            t_worst = time_data(global_worst_index + 1);

            % Depth disturbance rejection at worst moment
            depth_signal = dummy_out.logsout.get('z');
            depth_data   = depth_signal.Values.Data;
            if size(depth_data, 2) > 1; depth_data = depth_data(:,1); end

            [~, idx_worst_global] = min(abs(time_data - t_worst));
            window     = round(30 / mean(diff(time_data)));
            idx_start  = max(1,                  idx_worst_global - window);
            idx_end    = min(length(depth_data), idx_worst_global + window);
            depth_win  = depth_data(idx_start:idx_end);
            max_depth_error = max(abs(depth_win - tests(t).zf));

            test_metrics.t_worst       = t_worst;
            test_metrics.max_depth_err = max_depth_error;
            test_metrics.message       = sprintf('Most violent shift at t=%.1fs | Peak depth error: %.4fm', ...
                t_worst, max_depth_error);
            fprintf('Done. %s\n', test_metrics.message);

        elseif t == 2 || t == 4
            % --- Tests 2 & 4: Ascent / Descent Settling Time & Overshoot ---
            depth_signal = dummy_out.logsout.get('z');
            depth_data   = depth_signal.Values.Data;
            if size(depth_data, 2) > 1; depth_data = depth_data(:,1); end

            target_depth = tests(t).zf;
            start_depth  = tests(t).z0;
            
            % --- Calculate Overshoot ---
            if target_depth < start_depth
                % Ascent: Overshoot is the minimum depth reached
                peak_value = min(depth_data);
                overshoot_val = max(0, target_depth - peak_value);
            else
                % Descent: Overshoot is the maximum depth reached
                peak_value = max(depth_data);
                overshoot_val = max(0, peak_value - target_depth);
            end
            pct_overshoot = (overshoot_val / abs(target_depth - start_depth)) * 100;

            % --- Settling Time Logic ---
            pct_tol      = 0.02;
            tolerance    = pct_tol * abs(target_depth - start_depth);
            error_array         = abs(depth_data - target_depth);
            outside_tol_indices = find(error_array > tolerance);

            if isempty(outside_tol_indices)
                test_metrics.t_complete = 0;
                settle_msg = 'Settled at t=0.0s';
            elseif outside_tol_indices(end) == length(depth_data)
                test_metrics.t_complete = NaN;
                settle_msg = 'WARNING: Failed to settle';
            else
                test_metrics.t_complete = time_data(outside_tol_indices(end) + 1);
                settle_msg = sprintf('Settled at t=%.1fs', test_metrics.t_complete);
            end

            % Store and Print
            test_metrics.overshoot_m = overshoot_val;
            test_metrics.message = sprintf('%s | Overshoot: %.2fm (%.1f%%)', ...
                settle_msg, overshoot_val, pct_overshoot);
            
            fprintf('Done. %s\n', test_metrics.message);

        elseif t == 3
            % --- Test 3: Silent Windup ---
            % Steady-state depth hold under calm conditions.
            % Metric: RMS depth error and max absolute depth error over full run.
            depth_signal = dummy_out.logsout.get('z');
            depth_data   = depth_signal.Values.Data;
            if size(depth_data, 2) > 1; depth_data = depth_data(:,1); end

            % Discard first 60s of transient, analyze steady-state hold
            t_settle_idx = find(time_data >= 60, 1);
            ss_depth     = depth_data(t_settle_idx:end);

            rms_error = rms(ss_depth - tests(t).zf);
            max_error = max(abs(ss_depth - tests(t).zf));

            test_metrics.rms_depth_error = rms_error;
            test_metrics.max_depth_error = max_error;
            test_metrics.message         = sprintf('Depth hold (t>60s): RMS err=%.3fm | Max err=%.3fm', ...
                rms_error, max_error);
            fprintf('Done. %s\n', test_metrics.message);

        elseif t == 5 || t == 6
            % --- Tests 5 & 6: Combined Depth Hold + Heading Maneuver ---
            % Metrics: heading settling time + depth disturbance during maneuver.

            % --- Heading settling time ---
            heading_signal = dummy_out.logsout.get('yaw');
            heading_data   = heading_signal.Values.Data;
            time_data      = heading_signal.Values.Time;
            if size(heading_data, 2) > 1; heading_data = heading_data(:,1); end

            target_heading = tests(t).hf;
            pct_tol_h      = 0.02;
            tolerance_h = max(0.05, pct_tol_h * abs(target_heading - tests(t).h0));
            
            % Wrap heading error to [-pi, pi]
            heading_err = wrapToPi(heading_data - target_heading);
            outside_h   = find(abs(heading_err) > tolerance_h);

            if isempty(outside_h)
                test_metrics.t_heading_settle = 0;
                heading_msg = 'Heading settled immediately';
            elseif outside_h(end) == length(heading_data)
                test_metrics.t_heading_settle = NaN;
                heading_msg = 'WARNING: Heading never settled';
            else
                test_metrics.t_heading_settle = time_data(outside_h(end) + 1);
                heading_msg = sprintf('Heading settled at t=%.1fs', test_metrics.t_heading_settle);
            end

            % --- Depth disturbance during heading maneuver ---
            depth_signal = dummy_out.logsout.get('z');
            depth_data   = depth_signal.Values.Data;
            if size(depth_data, 2) > 1; depth_data = depth_data(:,1); end

            max_depth_error = max(abs(depth_data - tests(t).zf));

            test_metrics.max_depth_error = max_depth_error;
            test_metrics.message = sprintf('%s | Max depth coupling error: %.3fm', ...
                heading_msg, max_depth_error);
            fprintf('Done. %s\n', test_metrics.message);
        end

        % Save simulation output and analytics
        filename = sprintf('%s_Seed_%d.mat', strrep(tests(t).name, ' ', '_'), test_seeds(i));
        save(filename, 'dummy_out', 'test_metrics');
    end

    fprintf('\n');
end

disp('All Test Tiers Complete!');