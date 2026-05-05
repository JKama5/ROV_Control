% RunTests2.m
% full scope runner for AUV depth/course controller tests
% Chris Hunt & Jack Kamataris
% RBE 502 Final Project
clear workspace

mdl = 'AUVdepthHeadingControlWithRealisticWind';
load_system(mdl);

test_seeds = [111, 222, 333];
tests_to_run = 1:8;

% Wind and control mode enums
Simulink.defineIntEnumType('Wind', {'Off', 'Calm', 'Normal', 'Stormy'},...
    [-1; 1; 2; 3]);
Simulink.defineIntEnumType('ControlMode', {'Heading', 'Course'}, [0; 1]);

% shared PID gains
kp = 0.08;
ki = 0;
kd = 20;
CM = ControlMode.Course;

%% Tests
clear tests
tests(1) = struct('name', 'Wind Off', 'wind', Wind.Off, 'ctrl', CM, 'z0', 30, 'zf', 30, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(2) = struct('name', 'High Disturbance Maintaining Heading', 'wind', Wind.Stormy, 'ctrl', ControlMode.Heading, 'z0', 5, 'zf', 5, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(3) = struct('name', 'High Disturbance Maintaining Course', 'wind', Wind.Stormy, 'ctrl', CM, 'z0', 5, 'zf', 5, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(4) = struct('name', 'High Disturbance Maneuvering', 'wind', Wind.Stormy, 'ctrl', CM, 'z0', 5, 'zf', 5, 'h0', 0, 'hf', -60, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(5) = struct('name', 'The Ekman Corkscrew', 'wind', Wind.Stormy, 'ctrl', CM, 'z0', 1, 'zf', 50, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(6) = struct('name', 'The Surface Breach', 'wind', Wind.Stormy, 'ctrl', CM, 'z0', 100, 'zf', 5, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(7) = struct('name', 'Combined Accent and Maneuver', 'wind', Wind.Stormy, 'ctrl', CM, 'z0', 100, 'zf', 5, 'h0', 0, 'hf', -60, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests(8) = struct('name', 'The Silent Windup', 'wind', Wind.Calm, 'ctrl', CM, 'z0', 30, 'zf', 30, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);

%% RUN
for t = tests_to_run
    fprintf('=== %s ===\n', tests(t).name);
    for i = 1:length(test_seeds)
        seed = test_seeds(i);
        fprintf('  seed %d ... ', seed);
        sim_in = Simulink.SimulationInput(mdl);
        sim_in = setVariable(sim_in, 'active_seed', seed);
        sim_in = setVariable(sim_in, 'WindConditionsEnum', tests(t).wind);
        sim_in = setVariable(sim_in, 'CourseControlToggle', double(tests(t).ctrl));
        sim_in = setVariable(sim_in, 'Initial_depth', tests(t).z0);
        sim_in = setVariable(sim_in, 'Final_depth', tests(t).zf);
        sim_in = setVariable(sim_in, 'Initial_heading', tests(t).h0);
        sim_in = setVariable(sim_in, 'Final_heading', tests(t).hf);
        sim_in = setVariable(sim_in, 'Kpz', tests(t).Kpz);
        sim_in = setVariable(sim_in, 'Kiz', tests(t).Kiz);
        sim_in = setVariable(sim_in, 'Kdz', tests(t).Kdz);

        % set depth step block initial value
        depth_blk = find_system(mdl, 'BlockType', 'Step');
        sim_in = setBlockParameter(sim_in, depth_blk{1}, 'Before', num2str(tests(t).z0));
        sim_out = sim(sim_in);
        sim_out = setUserData(sim_out, tests(t));

        % pull current direction for plotting later
        dir_sig = sim_out.yout.get('Pre_Wrap_Current_Direction');
        dir_data = dir_sig.Values.Data;
        t_data = dir_sig.Values.Time;

        % basic metrics stub
        metrics = struct();
        metrics.test_name = tests(t).name;
        metrics.seed = seed;
        fname = sprintf('%s_Seed_%d.mat', strrep(tests(t).name, ' ', '_'), seed);
        save(fname, 'sim_out', 'metrics');
        fprintf('done\n');
    end
end
disp('All tests complete.');