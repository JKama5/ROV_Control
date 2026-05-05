% GeneratePlots.m
% Chris Hunt & Jack Kamataris
% RBE 502 Final Project

% run from the directory containing the .mat result files
% saves all figures as .png to a /figures/ subfolder

close all; clc; clear
mkdir('figures');

set(0, 'DefaultAxesFontSize', 12);
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultLineLineWidth', 1.5);
set(0, 'DefaultAxesBox', 'on');
set(0, 'DefaultFigureColor', 'w');

global seeds Test_names colors seed_labels

seed_labels = {'Seed 111', 'Seed 222', 'Seed 333'};

colors = [0.122 0.471 0.706;
          0.890 0.102 0.110;
          0.200 0.627 0.173];

seeds = [111, 222, 333];

Test_names = {'Wind_Off', ...
    'High_Disturbance_Maintaining_Heading', ...
    'High_Disturbance_Maintaining_Course', ...
    'High_Disturbance_Maneuvering', ...
    'The_Ekman_Corkscrew', ...
    'The_Surface_Breach', ...
    'Combined_Accent_and_Maneuver', ...
    'The_Silent_Windup'};

for i = 2:length(Test_names)
    clear inputParams globalT globalZ globalX globalY dir_sig dir_data
    global inputParams globalT globalZ globalX globalY dir_sig dir_data
    [inputParams, globalT, globalZ, globalX, globalY, dir_sig, dir_data] = extractData(i);
    [RMSE, RMSEeq, YmaxError, ZErrorBounds, ZOverShoot, ZSettleTime, YSettleTime] = NumericalResults(i);
    Test_names(i)

    data = {RMSE, RMSEeq, YmaxError, ZErrorBounds, ZOverShoot, ZSettleTime, YSettleTime};
    dataText = {'RMSE', 'RMSEeq', 'YmaxError', 'ZErrorBounds', 'ZOverShoot', 'ZSettleTime', 'YSettleTime'};

    for s = 1:length(seeds)
        if inputParams.hf ~= inputParams.h0
            [test, index] = min(abs(globalT(:, s) - YSettleTime(s)));
            XYbounds(s, :) = [1, round(1.25*index)];
        else
            XYbounds(s, :) = [1, size(globalT, 1)];
        end
    end
    bounds = [min(XYbounds(:, 1)), max(XYbounds(:, 2))];
    figure;
    basicXYPlot(i, bounds, YSettleTime, RMSEeq);
    myText = {};

    for j = 1:length(data)
        if ~isempty(data{j})
            valStr = mat2str(data{j}, 4);
            lineStr = sprintf('%s: %s', dataText{j}, valStr);
            myText{end+1} = lineStr;
        end
    end

    dim = [.92 0.5 0.8 0.3];
    annotation('textbox', dim, 'String', myText, 'BackgroundColor', 'white');
    hold off

    if inputParams.zf ~= inputParams.z0
        for s = 1:length(seeds)
            [test, index] = min(abs(globalT(:, s) - ZSettleTime(s)));
            Zbounds(s, :) = [1, round(1.25*index)];
        end
        bounds = [min(Zbounds(:, 1)), round(max(Zbounds(:, 2))*1.25)];
        figure;
        basicZTPlot(i, bounds, 1.25*max(ZSettleTime))
    end
end

for i=2:length(Test_names)
    clear inputParams globalT globalZ globalX globalY dir_sig dir_data globalYaw globalSpd globalDir
    global inputParams globalT globalZ  globalX globalY dir_sig dir_data globalYaw globalSpd globalDir
    
    [inputParams, globalT, globalZ, globalX, globalY, dir_sig, dir_data] = extractData(i);
    [RMSE, RMSEeq, YmaxError, ZErrorBounds, ZOverShoot, ZSettleTime, YSettleTime] = NumericalResults(i);
    disp(Test_names(i));
    
    % --- 1. Evaluate Active Plots & Dynamic Layout ---
    do_XY = true; 
    do_ZT = (inputParams.zf ~= inputParams.z0);
    do_VectorField = (i==2 || i==3);
    do_second_VectorField = false;
    
    % Calculate total plots to determine row count
    numPlots = do_XY + do_ZT + do_VectorField + do_second_VectorField;
    
    % Force at least 2 rows so the text box has good vertical space
    numRows = max(2, numPlots); 
    numCols = 3;
    
    % Open figure wide enough to support 3 columns
    fig = figure('Name', Test_names{i}, 'Position', [100, 100, 1200, 600]); 
    currentRow = 1;
    
    % --- 2. Generate Subplots ---
    
    % Plot 1: XY Plot
    if do_XY
        % Calculate bounds
        for s=1:length(seeds)
            if inputParams.hf~=inputParams.h0
                [~,index] = min(abs(globalT(:,s)-YSettleTime(s)));
                XYbounds(s,:)=[1,round(1.25*index)];
            else
                XYbounds(s,:)=[1,size(globalT,1)];
            end
        end
        XY_bounds = [min(XYbounds(:,1)), max(XYbounds(:,2))];
        
        if numPlots == 1
            idx = []; for r=1:numRows; idx = [idx, (r-1)*numCols+1, (r-1)*numCols+2]; end
        else
            idx = [(currentRow-1)*numCols+1, (currentRow-1)*numCols+2];
        end
        
        subplot(numRows, numCols, idx);
        basicXYPlot(i, XY_bounds, YSettleTime, RMSEeq);
        
        currentRow = currentRow + (numPlots > 1); 
    end
    
    % Plot 2: ZT Plot
    if do_ZT
        for s=1:length(seeds)
            [~,index] = min(abs(globalT(:,s)-ZSettleTime(s)));
            Zbounds(s,:)=[1,round(1.25*index)];
        end
        ZT_bounds=[min(Zbounds(:,1)),round(max(Zbounds(:,2))*1.25)];
        
        idx = [(currentRow-1)*numCols+1, (currentRow-1)*numCols+2];
        subplot(numRows, numCols, idx);
        basicZTPlot(i, ZT_bounds, 1.25*max(ZSettleTime));
        
        currentRow = currentRow + 1;
    end
    
    % Plot 3: First Vector Field
    if do_VectorField
        idx = [(currentRow-1)*numCols+1, (currentRow-1)*numCols+2];
        subplot(numRows, numCols, idx);
        basicVectorPlot(i, XY_bounds, 1); 
        currentRow = currentRow + 1;
    end
    
    % Plot 4: Second Vector Field
    if do_second_VectorField
        idx = [(currentRow-1)*numCols+1, (currentRow-1)*numCols+2];
        subplot(numRows, numCols, idx);
        basicVectorPlot(i, [32260,32340], 1);
        currentRow = currentRow + 1;
    end
    
    % --- 3. Text Box (Column 3) ---
    data={RMSE, RMSEeq, YmaxError, ZErrorBounds, ZOverShoot, ZSettleTime, YSettleTime};
    dataText={'RMSE', 'RMSEeq', 'YmaxError', 'ZErrorBounds', 'ZOverShoot', 'ZSettleTime', 'YSettleTime'};
    
    myText = {};
    for j = 1:length(data)
        if ~isempty(data{j})
            myText{end+1} = sprintf('%s:', dataText{j});
            
            if contains(dataText{j}, 'Time')
                unit = 's';
            elseif strcmp(dataText{j}, 'RMSEeq')
                unit = ''; 
            else
                unit = 'm'; 
            end
            
            currentData = data{j};
            
            for s = 1:length(seeds)
                if size(currentData, 1) == 1
                    myText{end+1} = sprintf('    Seed %d: %.3g%s', seeds(s), currentData(1, s), unit);
                elseif size(currentData, 1) == 2
                    if strcmp(dataText{j}, 'ZErrorBounds')
                        myText{end+1} = sprintf('    Seed %d: %.3g%s to %.3g%s', ...
                            seeds(s), currentData(1, s), unit, currentData(2, s), unit);
                    elseif strcmp(dataText{j}, 'RMSEeq')
                        myText{end+1} = sprintf('    Seed %d: slope=%.3g, int=%.3g', ...
                            seeds(s), currentData(1, s), currentData(2, s));
                    else
                        myText{end+1} = sprintf('    Seed %d: [%.3g, %.3g]', ...
                            seeds(s), currentData(1, s), currentData(2, s));
                    end
                end
            end
            myText{end+1} = ' ';
        end
    end
    
    textIdx = 3:numCols:(numRows*numCols);
    axText = subplot(numRows, numCols, textIdx);
    axis(axText, 'off'); 
    
    text(0.5, 0.5, myText, 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'BackgroundColor', 'white', ...
        'EdgeColor', 'k', ...
        'Margin', 10, ...
        'Interpreter', 'none');
        
    % --- 4. Save Figure to Folder ---
    % Ensure the 'figures' directory exists so MATLAB doesn't throw a path error
    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    
    % Construct the file path using the Test_names cell array
    fileName = sprintf('%s.png', Test_names{i});
    fullFilePath = fullfile('figures', fileName);
    
    % Export as a high-resolution 300 DPI PNG (requires MATLAB R2020a or newer)
    % (If you are on an older version, replace this line with: saveas(fig, fullFilePath); )
    exportgraphics(fig, fullFilePath, 'Resolution', 300);
    
    disp(['Saved: ', fullFilePath]);
        
end

function [t, d] = extractSignal(sim_out, name)
    sig = sim_out.logsout.get(name);
    t = sig.Values.Time;
    d = sig.Values.Data;
    if size(d, 2) > 1
        d = d(:, 1);
    end
end

function [inputParams, t, z, x, y, dir_sig, dir_data] = extractData(TestNum)
    global seeds Test_names globalYaw globalSpd globalDir
    for i = 1:length(seeds)
        fname = sprintf('%s_Seed_%d.mat', Test_names{TestNum}, seeds(i));
        load(fname, 'dummy_out', 'test_metrics');
        [t(:, i), z(:, i)] = extractSignal(dummy_out, 'z');
        [~, x(:, i)] = extractSignal(dummy_out, 'x');
        [~, y(:, i)] = extractSignal(dummy_out, 'y');
        dir_sig(i) = dummy_out.yout.get('Pre_Wrap_Current_Direction');
        dir_data(:, i) = rad2deg(dir_sig(i).Values.Data);
        inputParams = dummy_out.SimulationMetadata.UserData;

        [t(:,i), z(:,i)]   = extractSignal(dummy_out, 'z');
        [~,x(:,i)] = extractSignal(dummy_out,'x');
        [~,y(:,i)] = extractSignal(dummy_out,'y');
        
        [~, globalYaw(:,i)] = extractSignal(dummy_out, 'yaw');
        dir_sig(i) = dummy_out.logsout.get('Current_Direction');

        globalDir(:,i) = dir_sig(i).Values.Data; % Keep in RADIANS for the math
        dir_data(:,i) = rad2deg(dir_sig(i).Values.Data); 
        
        spd_sig = dummy_out.logsout.get('Current_Speed');
        globalSpd(:,i) = spd_sig.Values.Data;
        inputParams=dummy_out.SimulationMetadata.UserData;
    end
end

function [RMSE, RMSEeq, YmaxError, ZErrorBounds, ZOverShoot, ZSettleTime, YSettleTime] = NumericalResults(testNum)
    global globalX globalY globalZ globalT inputParams seeds
    [RMSE, RMSEeq, YmaxError, ZErrorBounds, ZOverShoot, ZSettleTime, YSettleTime] = deal([]);
    RMSList = [3, 5, 6, 8];
    RMSEList = [4, 7];
    YmaxErrorList = [2:8];
    ZErrorBoundsList = [2, 3, 4, 8];
    ZOverShootList = [5, 6, 7];
    ZSettleTimeList = ZOverShootList;
    YSettleTimeList = RMSEList;

    for i = 1:length(seeds)
        y = globalY(:, i);
        x = globalX(:, i);
        z = globalZ(:, i);
        t = globalT(:, i);

        if ismember(testNum, RMSList)
            RMSE(:, i) = rmse(y, ones(size(y))*mean(y));
        elseif ismember(testNum, RMSEList)
            m = tand(inputParams.hf);
            b = mean(y - m*x);
            p = m.*x + b;
            RMSE(:, i) = rmse(y, p);
            RMSEeq(:, i) = [m; b];
            ygoal = hypot(x, y)*sind(inputParams.hf);

            if ismember(testNum, YmaxErrorList)
                YmaxError(:, i) = abs(ygoal(length(ygoal)) - y(length(y)));
            end
        end

        if ismember(testNum, YmaxErrorList) && ~ismember(testNum, RMSEList)
            YmaxError(:, i) = max(abs(y));
        end

        if ismember(testNum, ZErrorBoundsList)
            ZErrorBounds(:, i) = [min(z) - inputParams.zf; max(z) - inputParams.zf];
        end

        if ismember(testNum, ZOverShootList)
            if inputParams.z0 < inputParams.zf
                ZOverShoot(:, i) = abs(max(z) - inputParams.zf);
            else
                ZOverShoot(:, i) = abs(min(z) - inputParams.zf);
            end
        end

        if ismember(testNum, ZSettleTimeList)
            threshold = .5;
            PercentThreshold = threshold/z(length(z));
            data = stepinfo(z, t, z(length(z)), 'SettlingTimeThreshold', PercentThreshold);
            ZSettleTime(:, i) = data.SettlingTime;
        end

        if ismember(testNum, YSettleTimeList)
            threshold = RMSE(:, i);
            error = abs(p - y);
            unsettled_indices = find(error > threshold);
            if isempty(unsettled_indices)
                YSettleTime(i) = t(1);
            else
                last_unsettled_idx = unsettled_indices(end);
                if last_unsettled_idx < length(t)
                    YSettleTime(i) = t(last_unsettled_idx + 1);
                else
                    YSettleTime(i) = NaN;
                end
            end
        end
    end
end

function basicXYPlot(testNum, bounds, settleTime, RMSEeq)
    global globalX globalY globalT colors seed_labels inputParams Test_names
    hold on
    for i = 1:3
        x = globalX(bounds(1):bounds(2), i);
        y = globalY(bounds(1):bounds(2), i);
        plot(x, y, 'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', seed_labels{i});
        if ~isempty(settleTime)
            [~, settleIndex] = min(abs(globalT(:, i) - settleTime(i)));
            xline(x(settleIndex), '--', 'Color', colors(i, :), 'DisplayName', 'Course settled within RMSE of mean path')
        end
    end
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    tName = replace(Test_names{testNum}, "_", " ");
    pName = sprintf('%s XY Position Plot', tName);
    title(pName);
    grid on;
    ygoal = x(:, 1)*tand(inputParams.hf);
    plot(x, ygoal, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target path');
    if ~isempty(RMSEeq)
        Pathlabel = sprintf('Mean Path; Mean Error: %.2f (m)', RMSEeq(2));
        plot(x, RMSEeq(1, 1)*x + mean(RMSEeq(2, :)), 'm--', 'DisplayName', Pathlabel);
    end
    legend('Location', 'best');
    
    % --- Dynamic Subtitle Logic ---
    total_length = size(globalX, 1);
    if bounds(1) > 1 || bounds(2) < total_length
        start_m = globalX(bounds(1), 1);
        end_m   = globalX(bounds(2), 1);
        total_m = globalX(end, 1);
        subtitle(sprintf('Plotted from x = %.0fm to %.0fm out of %.0fm', start_m, end_m, total_m));
    end
    % ------------------------------
    
    grid on;
    
    % Add a target path reference line
    ygoal=x(:,1)*tand(inputParams.hf);
    plot(x,ygoal, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target path');
    if ~isempty(RMSEeq)
        Pathlable=sprintf('Mean Path; Mean Error: %.2f (m)',RMSEeq(2));
        plot(x,RMSEeq(1,1)*x+mean(RMSEeq(2,:)), 'm--', 'DisplayName', Pathlable);
    end
    legend('Location', 'best');
    
    % hold off
    % saveas(fig8, 'figures/Test8_TrajectoryOverlay.png');
    % fprintf('Saved Trajectory Overlay Plot\n');
end

function basicZTPlot(testNum, bounds, settleTime)
    global globalZ globalT colors seed_labels inputParams Test_names
    hold on
    for i = 1:3
        z = globalZ(bounds(1):bounds(2), i);
        t = globalT(bounds(1):bounds(2), i);
        plot(t, z, 'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', seed_labels{i});
        xline(settleTime, '--', 'Color', colors(i, :), 'DisplayName', 'depth settled within 0.5m')
    end
    xlabel('Time (s)');
    ylabel('Z Position (m)');
    tName = replace(Test_names{testNum}, "_", " ");
    pName = sprintf('%s Z Position vs Time Plot', tName);
    title(pName);
    grid on;

    
    % --- Dynamic Subtitle Logic ---
    total_length = size(globalT, 1);
    if bounds(1) > 1 || bounds(2) < total_length
        start_s = globalT(bounds(1), 1);
        end_s   = globalT(bounds(2), 1);
        total_s = globalT(end, 1);
        subtitle(sprintf('Plotted from t = %.0fs to %.0fs out of %.0fs', start_s, end_s, total_s));
    end
    % ------------------------------
    
    grid on;
    
    % Add a target path reference line
    yline(inputParams.zf, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target Depth');
    legend('Location', 'best');
    hold off
end

%% Test 1: The Broadside Wall
fig1 = figure('Position', [100 100 900 500]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1a = nexttile; hold on;
ax1b = nexttile; hold on;

for i = 1:3
    fname = sprintf('The_Broadside_Wall_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    dir_sig = dummy_out.yout.get('Pre_Wrap_Current_Direction');
    t_dir = dir_sig.Values.Time;
    dir_data = rad2deg(dir_sig.Values.Data);

    plot(ax1a, t/3600, z, 'Color', colors(i, :), 'DisplayName', seed_labels{i});
    plot(ax1b, t_dir/3600, dir_data, 'Color', colors(i, :), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_worst')
        xline(ax1a, test_metrics.t_worst/3600, '--', 'Color', colors(i, :), 'LineWidth', 1, 'HandleVisibility', 'off');
        xline(ax1b, test_metrics.t_worst/3600, '--', 'Color', colors(i, :), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(ax1a, 5, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel(ax1a, 'Depth z (m)');
title(ax1a, 'Test 1: The Broadside Wall — Depth Response');
legend(ax1a, 'Location', 'southeast');
ylim(ax1a, [4.8 6.2]);
xlim(ax1a, [0 1]);
text(ax1a, 0.95, 4.83, 'Target: 5m', 'HorizontalAlignment', 'right', 'FontSize', 10);

ylabel(ax1b, 'Current Direction (deg)');
xlabel(ax1b, 'Time (hr)');
title(ax1b, 'Current Direction — accumulated (dashed = worst shift moment)');
xlim(ax1b, [0 1]);

saveas(fig1, 'figures/Test1_BroadsideWall.png');
fprintf('Saved Test 1\n');

%% Test 2: The Ekman Corkscrew
fig2 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Ekman_Corkscrew_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i, :), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_complete') && ~isnan(test_metrics.t_complete)
        xline(test_metrics.t_complete, '--', 'Color', colors(i, :), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(50, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(50 + 0.98, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(50 - 0.98, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
text(400, 45, 'Target: 50m', 'HorizontalAlignment', 'left', 'FontSize', 10);
text(400, 42, '±2% band', 'HorizontalAlignment', 'left', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 2: The Ekman Corkscrew — Descent 1m \rightarrow 50m (dashed = settled)');
legend('Location', 'southeast');
xlim([0 600]);
ylim([0 60]);

saveas(fig2, 'figures/Test2_EkmanCorkscrew.png');
fprintf('Saved Test 2\n');

%% Test 3: The Silent Windup
fig3 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Silent_Windup_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i, :), 'DisplayName', seed_labels{i});
end

yline(30, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(30 + 0.096, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(30 - 0.096, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
text(300, 22, 'Target: 30m', 'HorizontalAlignment', 'left', 'FontSize', 10);
text(300, 19, 'RMS band', 'HorizontalAlignment', 'left', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 3: The Silent Windup — Calm Depth Hold at 30m');
legend('Location', 'southeast');
xlim([0 600]);
ylim([5 35]);

saveas(fig3, 'figures/Test3_SilentWindup.png');
fprintf('Saved Test 3\n');

%% Test 4: The Surface Breach
fig4 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Surface_Breach_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i, :), 'DisplayName', seed_labels{i});
end

yline(5, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(5 + 1.9, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(5 - 1.9, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
text(400, 25, 'Target: 5m', 'HorizontalAlignment', 'left', 'FontSize', 10);
text(400, 20, '±2% band', 'HorizontalAlignment', 'left', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 4: The Surface Breach — Ascent 100m \rightarrow 5m (Stormy)');
legend('Location', 'northeast');
xlim([0 600]);
ylim([-20 120]);

saveas(fig4, 'figures/Test4_SurfaceBreach.png');
fprintf('Saved Test 4\n');

%% Test 5: The Operational Envelope
fig5 = figure('Position', [100 100 900 600]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax5a = nexttile; hold on;
ax5b = nexttile; hold on;

for i = 1:3
    fname = sprintf('The_Operational_Envelope_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    [~, yaw] = extractSignal(dummy_out, 'yaw');

    plot(ax5a, t, yaw, 'Color', colors(i, :), 'DisplayName', seed_labels{i});
    plot(ax5b, t, z, 'Color', colors(i, :), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_heading_settle') && ~isnan(test_metrics.t_heading_settle)
        xline(ax5a, test_metrics.t_heading_settle, '--', 'Color', colors(i, :), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(ax5a, -60, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax5a, 3900, -57, 'Target: -60 rad', 'HorizontalAlignment', 'right', 'FontSize', 10);
ylabel(ax5a, 'Yaw \psi (rad)');
title(ax5a, 'Test 5: Operational Envelope — Heading 0 \rightarrow -60 rad');
legend(ax5a, 'Location', 'southeast');
xlim(ax5a, [0 4000]);
ylim(ax5a, [-65 5]);

yline(ax5b, 10, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax5b, 3900, 10.3, 'Target: 10m', 'HorizontalAlignment', 'right', 'FontSize', 10);
ylabel(ax5b, 'Depth z (m)');
xlabel(ax5b, 'Time (s)');
title(ax5b, 'Depth Coupling During Heading Maneuver');
xlim(ax5b, [0 4000]);
ylim(ax5b, [0 12]);

saveas(fig5, 'figures/Test5_OperationalEnvelope.png');
fprintf('Saved Test 5\n');

%% Test 6: The Deep Freeze
fig6 = figure('Position', [100 100 900 600]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax6a = nexttile; hold on;
ax6b = nexttile; hold on;

for i = 1:3
    fname = sprintf('The_Deep_Freeze_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    [~, yaw] = extractSignal(dummy_out, 'yaw');

    plot(ax6a, t, yaw, 'Color', colors(i, :), 'DisplayName', seed_labels{i});
    plot(ax6b, t, z, 'Color', colors(i, :), 'DisplayName', seed_labels{i});
end

yline(ax6a, -60, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax6a, 3900, -57, 'Target: -60 rad', 'HorizontalAlignment', 'right', 'FontSize', 10);
ylabel(ax6a, 'Yaw \psi (rad)');
title(ax6a, 'Test 6: The Deep Freeze — Heading 0 \rightarrow -60 rad (Stormy, 100m)');
legend(ax6a, 'Location', 'southeast');
xlim(ax6a, [0 4000]);
ylim(ax6a, [-65 5]);

yline(ax6b, 100, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax6b, 3900, 100.8, 'Target: 100m', 'HorizontalAlignment', 'right', 'FontSize', 10);
ylabel(ax6b, 'Depth z (m)');
xlabel(ax6b, 'Time (s)');
title(ax6b, 'Depth Hold Under Stormy Conditions + Heading Maneuver');
xlim(ax6b, [0 4000]);
ylim(ax6b, [55 115]);

saveas(fig6, 'figures/Test6_DeepFreeze.png');
fprintf('Saved Test 6\n');

%% Test 7: XY trajectory, heading, and current (equal axis)
fig7 = figure('Position', [100 100 900 900]);
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

num_vectors = 45;
num_y_lines = 5;

for i = 1:3
    ax = nexttile; hold on;

    fname = sprintf('The_Broadside_Wall_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out');

    [t_xy, x] = extractSignal(dummy_out, 'x');
    [~, y] = extractSignal(dummy_out, 'y');
    [~, yaw] = extractSignal(dummy_out, 'yaw');

    dir_sig = dummy_out.logsout.get('Current_Direction');
    t_dir = dir_sig.Values.Time;
    theta = dir_sig.Values.Data;

    spd_sig = dummy_out.logsout.get('Current_Speed');
    spd = spd_sig.Values.Data;

    theta_interp = interp1(t_dir, theta, t_xy, 'linear', 'extrap');
    spd_interp = interp1(t_dir, spd, t_xy, 'linear', 'extrap');

    idx = round(linspace(1, length(t_xy), num_vectors));
    idxLess = round(linspace(1, length(t_xy), round(num_vectors/3)));

    x_quiv = x(idx);
    theta_quiv = theta_interp(idx);
    spd_quiv = spd_interp(idx);

    x_quiv_less = x(idxLess);
    y_quiv_less = y(idxLess);
    yaw_quiv = yaw(idxLess);

    x_span = max(x) - min(x);
    y_band = x_span / 12;
    y_min = -y_band;
    y_max = y_band;

    [X_grid, Y_grid] = meshgrid(x_quiv, linspace(y_min, y_max, num_y_lines));
    Theta_grid = repmat(theta_quiv(:)', num_y_lines, 1);
    Spd_grid = repmat(spd_quiv(:)', num_y_lines, 1);

    arrow_L = x_span / 30;

    U_curr = (arrow_L * 0.8) .* Spd_grid .* cos(Theta_grid);
    V_curr = (arrow_L * 0.8) .* Spd_grid .* sin(Theta_grid);

    quiver(ax, X_grid, Y_grid, U_curr, V_curr, 0, 'Color', [0.4 0.6 0.9], ...
        'LineWidth', 1, 'MaxHeadSize', 0.5, 'DisplayName', 'Current Direction');

    plot(ax, x, y, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');

    u_yaw = arrow_L .* cos(yaw_quiv);
    v_yaw = arrow_L .* sin(yaw_quiv);

    quiver(ax, x_quiv_less, y_quiv_less, u_yaw, v_yaw, 0, 'Color', 'r', ...
        'LineWidth', 1.5, 'MaxHeadSize', 1.5, 'DisplayName', 'Heading');

    ylabel(ax, 'Y (m)');
    title(ax, sprintf('Test 1: True Angle Vectors (%s)', seed_labels{i}));

    axis(ax, 'equal');
    ylim(ax, [y_min*1.2, y_max*1.2]);
    grid(ax, 'on');

    if i == 3
        xlabel(ax, 'X (m)');
        legend(ax, 'Location', 'best');
    end
end

saveas(fig7, 'figures/Test7_VectorField_EqualAxis.png');
fprintf('Saved Test 7\n');

%% Test 8: Y vs X trajectory overlay
fig8 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Broadside_Wall_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out');

    [~, x] = extractSignal(dummy_out, 'x');
    [~, y] = extractSignal(dummy_out, 'y');

    plot(x, y, 'Color', colors(i, :), 'LineWidth', 1.5, 'DisplayName', seed_labels{i});
end

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Test 1: The Broadside Wall — Cross-Track Drift');
legend('Location', 'best');
grid on;
yline(0, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

saveas(fig8, 'figures/Test8_TrajectoryOverlay.png');
fprintf('Saved Test 8\n');

fprintf('\nAll figures saved to /figures/\n');
function basicVectorPlot(testNum, bounds, seedIndex)
    global globalX globalY globalYaw globalSpd globalDir colors seed_lables Test_names
    
    hold on;
    % Adjust these to tune the visual density of the arrows
    num_vectors = 45; 
    num_heading_vectors = round(num_vectors / 3);
    num_y_lines = 5;  
    
    % --- 1. Extract Bounded Data ---
    x_b   = globalX(bounds(1):bounds(2), seedIndex);
    y_b   = globalY(bounds(1):bounds(2), seedIndex);
    
    % CONVERT TO RADIANS IMMEDIATELY
    yaw_b = deg2rad(globalYaw(bounds(1):bounds(2), seedIndex)); 
    spd_b = globalSpd(bounds(1):bounds(2), seedIndex);
    dir_b = deg2rad(globalDir(bounds(1):bounds(2), seedIndex));
    
    % --- 2. Calculate Bins for Averaging ---
    % Create bin edges based on the number of requested vectors
    bin_edges_curr = round(linspace(1, length(x_b), num_vectors + 1));
    bin_edges_head = round(linspace(1, length(x_b), num_heading_vectors + 1));
    
    % Initialize arrays for the averaged values
    [x_curr_avg, spd_curr_avg, dir_curr_avg] = deal(zeros(1, num_vectors));
    [x_head_avg, y_head_avg, yaw_head_avg]   = deal(zeros(1, num_heading_vectors));
    
    % --- 3. Average Current Data (Circular Mean for Angles) ---
    for k = 1:num_vectors
        idx_range = bin_edges_curr(k):(bin_edges_curr(k+1)-1);
        if isempty(idx_range); continue; end
        
        x_curr_avg(k)   = mean(x_b(idx_range));
        spd_curr_avg(k) = mean(spd_b(idx_range));
        % Circular mean for current direction
        dir_curr_avg(k) = atan2(mean(sin(dir_b(idx_range))), mean(cos(dir_b(idx_range))));
    end
    
    % --- 4. Average Heading Data (Circular Mean for Angles) ---
    for k = 1:num_heading_vectors
        idx_range = bin_edges_head(k):(bin_edges_head(k+1)-1);
        if isempty(idx_range); continue; end
        
        x_head_avg(k) = mean(x_b(idx_range));
        y_head_avg(k) = mean(y_b(idx_range));
        % Circular mean for ROV heading
        yaw_head_avg(k) = atan2(mean(sin(yaw_b(idx_range))), mean(cos(yaw_b(idx_range))));
    end
    
    % --- 5. Dynamic Scaling Calculations ---
    x_min = min(x_b);
    x_max = max(x_b);
    x_span = x_max - x_min;
    if x_span == 0; x_span = 1; end % Prevent division by zero
    
    y_min_data = min(y_b);
    y_max_data = max(y_b);
    
    % Create a vertical band that frames the ACTUAL data properly
    y_buffer = x_span / 12; 
    y_min = y_min_data - y_buffer; 
    y_max = y_max_data + y_buffer;
    
    arrow_L = x_span / 30;
    
    % --- 6. Plotting ---
    % Current Field 
    [X_grid, Y_grid] = meshgrid(x_curr_avg, linspace(y_min, y_max, num_y_lines));
    Theta_grid = repmat(dir_curr_avg(:)', num_y_lines, 1);
    Spd_grid   = repmat(spd_curr_avg(:)', num_y_lines, 1);
    
    U_curr = (arrow_L * 0.8) .* Spd_grid .* cos(Theta_grid);
    V_curr = (arrow_L * 0.8) .* Spd_grid .* sin(Theta_grid);
    
    % Plot current (Light Blue)
    quiver(X_grid, Y_grid, U_curr, V_curr, 0, 'Color', [0.3010, 0.7450, 0.9330], ...
           'LineWidth', 1, 'MaxHeadSize', 0.5, 'DisplayName', 'Current Direction');
           
    % Trajectory (Black)
    plot(x_b, y_b, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory ');
    
    % Heading Vectors (Red)
    u_yaw = arrow_L .* cos(yaw_head_avg);
    v_yaw = arrow_L .* sin(yaw_head_avg);
    quiver(x_head_avg, y_head_avg, u_yaw, v_yaw, 0, 'Color', 'r', ...
           'LineWidth', 1.5, 'MaxHeadSize', 1.5, 'DisplayName', 'Heading ');
           
    % --- 7. Formatting ---
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    
    tName = replace(Test_names{testNum}, "_", " ");
    title(sprintf('%s Current Direction and Heading Overlay', tName));
    subtitle(sprintf('(%s)',seed_lables{seedIndex}))
    
    axis equal; 
    
    % FORCE the limits after 'axis equal' to ensure nothing is clipped
    xlim([x_min - (x_span*0.02), x_max + (x_span*0.02)]);
    ylim([y_min, y_max]);
    
    grid on;
    legend('Location', 'best');
    
    % --- Dynamic Subtitle ---
    total_length = size(globalX, 1);
    if bounds(1) > 1 || bounds(2) < total_length
        start_m = globalX(bounds(1), 1);
        end_m   = globalX(bounds(2), 1);
        total_m = globalX(end, 1);
        subtitle(sprintf('Averaged over %.1fm to %.1fm out of %.1fm', start_m, end_m, total_m));
    end
end
