% =========================================================
% GeneratePlots.m
% Publication-quality plots for all 6 test cases
% Chris Hunt, Jack Kamataris — RBE502 ROV Control Project
%
% Run from the directory containing the .mat result files.
% Saves all figures as .png to a /figures/ subfolder.
% =========================================================

close all; clc;
mkdir('figures');

% =========================================================
% PLOT STYLE DEFAULTS
% =========================================================
set(0, 'DefaultAxesFontSize',    12);
set(0, 'DefaultAxesFontName',    'Arial');
set(0, 'DefaultLineLineWidth',   1.5);
set(0, 'DefaultAxesBox',         'on');
set(0, 'DefaultFigureColor',     'w');

colors = [0.122 0.471 0.706;   % blue   - seed 111
          0.890 0.102 0.110;   % red    - seed 222
          0.200 0.627 0.173];  % green  - seed 333
seed_labels = {'Seed 111', 'Seed 222', 'Seed 333'};
seeds = [111, 222, 333];

% Helper: extract timeseries data safely
function [t, d] = extractSignal(sim_out, name)
    sig = sim_out.logsout.get(name);
    t   = sig.Values.Time;
    d   = sig.Values.Data;
    if size(d, 2) > 1; d = d(:,1); end
end

% =========================================================
% TEST 1: THE BROADSIDE WALL
% Depth hold at 5m under stormy current, show worst shift
% =========================================================
fig1 = figure('Position', [100 100 900 500]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile;
hold on;
ax2 = nexttile;
hold on;

for i = 1:3
    fname = sprintf('The_Broadside_Wall_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z]   = extractSignal(dummy_out, 'z');
    dir_sig  = dummy_out.yout.get('Pre_Wrap_Current_Direction');
    t_dir    = dir_sig.Values.Time;
    dir_data = rad2deg(dir_sig.Values.Data);

    plot(ax1, t/3600, z,        'Color', colors(i,:), 'DisplayName', seed_labels{i});
    plot(ax2, t_dir/3600, dir_data, 'Color', colors(i,:), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_worst')
        xline(ax1, test_metrics.t_worst/3600, '--', 'Color', colors(i,:), ...
            'LineWidth', 1, 'HandleVisibility', 'off');
        xline(ax2, test_metrics.t_worst/3600, '--', 'Color', colors(i,:), ...
            'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(ax1, 5, 'k--', 'Target: 5m', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel(ax1, 'Depth z (m)');
title(ax1, 'Test 1: The Broadside Wall — Depth Response');
legend(ax1, 'Location', 'best');
ylim(ax1, [4 6]);

ylabel(ax2, 'Current Direction (deg)');
xlabel(ax2, 'Time (hr)');
title(ax2, 'Current Direction (dashed = worst shift moment)');

saveas(fig1, 'figures/Test1_BroadsideWall.png');
fprintf('Saved Test 1\n');

% =========================================================
% TEST 2: THE EKMAN CORKSCREW
% Depth descent 1m -> 50m, show settling time
% =========================================================
fig2 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Ekman_Corkscrew_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');

    plot(t, z, 'Color', colors(i,:), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_complete') && ~isnan(test_metrics.t_complete)
        xline(test_metrics.t_complete, '--', 'Color', colors(i,:), ...
            'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

% 2% tolerance band around 50m target (49m range)
yline(50,        'k--', 'Target',     'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(50 - 0.98, 'k:',  '+2% band',  'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(50 + 0.98, 'k:',  '-2% band',  'LineWidth', 1.0, 'HandleVisibility', 'off');

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 2: The Ekman Corkscrew — Descent 1m \rightarrow 50m (dashed = settled)');
legend('Location', 'best');
xlim([0 600]);

saveas(fig2, 'figures/Test2_EkmanCorkscrew.png');
fprintf('Saved Test 2\n');

% =========================================================
% TEST 3: THE SILENT WINDUP
% Calm depth hold at 30m, steady-state error analysis
% =========================================================
fig3 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Silent_Windup_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i,:), 'DisplayName', seed_labels{i});
end

yline(30,    'k--', 'Target: 30m', 'LineWidth', 1.2, 'HandleVisibility', 'off');
% RMS error band (0.096m)
yline(30 + 0.096, 'k:', 'RMS band', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(30 - 0.096, 'k:', '',         'LineWidth', 1.0, 'HandleVisibility', 'off');

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 3: The Silent Windup — Calm Depth Hold at 30m');
legend('Location', 'best');
xlim([0 600]);

saveas(fig3, 'figures/Test3_SilentWindup.png');
fprintf('Saved Test 3\n');

% =========================================================
% TEST 4: THE SURFACE BREACH
% Ascent 100m -> 5m under stormy conditions
% =========================================================
fig4 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Surface_Breach_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i,:), 'DisplayName', seed_labels{i});
end

yline(5,      'k--', 'Target: 5m',  'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(5 - 1.9,'k:',  '+2% band',   'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(5 + 1.9,'k:',  '-2% band',   'LineWidth', 1.0, 'HandleVisibility', 'off');

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 4: The Surface Breach — Ascent 100m \rightarrow 5m (Stormy)');
legend('Location', 'best');

saveas(fig4, 'figures/Test4_SurfaceBreach.png');
fprintf('Saved Test 4\n');

% =========================================================
% TEST 5: THE OPERATIONAL ENVELOPE
% Depth hold 10m + heading maneuver 0 -> -60 rad
% =========================================================
fig5 = figure('Position', [100 100 900 600]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
ax5a = nexttile; hold on;
ax5b = nexttile; hold on;

for i = 1:3
    fname = sprintf('The_Operational_Envelope_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z]   = extractSignal(dummy_out, 'z');
    [~, yaw] = extractSignal(dummy_out, 'yaw');

    plot(ax5a, t, yaw,  'Color', colors(i,:), 'DisplayName', seed_labels{i});
    plot(ax5b, t, z,    'Color', colors(i,:), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_heading_settle') && ~isnan(test_metrics.t_heading_settle)
        xline(ax5a, test_metrics.t_heading_settle, '--', 'Color', colors(i,:), ...
            'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(ax5a, -60,   'k--', 'Target: -60 rad', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(ax5b,  10,   'k--', 'Target: 10m',     'LineWidth', 1.2, 'HandleVisibility', 'off');

ylabel(ax5a, 'Yaw \psi (rad)');
title(ax5a, 'Test 5: Operational Envelope — Heading 0 \rightarrow -60 rad');
legend(ax5a, 'Location', 'best');

ylabel(ax5b, 'Depth z (m)');
xlabel(ax5b, 'Time (s)');
title(ax5b, 'Depth Coupling During Heading Maneuver');

saveas(fig5, 'figures/Test5_OperationalEnvelope.png');
fprintf('Saved Test 5\n');

% =========================================================
% TEST 6: THE DEEP FREEZE
% Depth hold 100m + heading maneuver 0 -> -60 rad, stormy
% =========================================================
fig6 = figure('Position', [100 100 900 600]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
ax6a = nexttile; hold on;
ax6b = nexttile; hold on;

for i = 1:3
    fname = sprintf('The_Deep_Freeze_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z]   = extractSignal(dummy_out, 'z');
    [~, yaw] = extractSignal(dummy_out, 'yaw');

    plot(ax6a, t, yaw, 'Color', colors(i,:), 'DisplayName', seed_labels{i});
    plot(ax6b, t, z,   'Color', colors(i,:), 'DisplayName', seed_labels{i});
end

yline(ax6a, -60,  'k--', 'Target: -60 rad', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(ax6b, 100,  'k--', 'Target: 100m',    'LineWidth', 1.2, 'HandleVisibility', 'off');

ylabel(ax6a, 'Yaw \psi (rad)');
title(ax6a, 'Test 6: The Deep Freeze — Heading 0 \rightarrow -60 rad (Stormy, 100m)');
legend(ax6a, 'Location', 'best');

ylabel(ax6b, 'Depth z (m)');
xlabel(ax6b, 'Time (s)');
title(ax6b, 'Depth Instability Under Stormy Conditions');

saveas(fig6, 'figures/Test6_DeepFreeze.png');
fprintf('Saved Test 6\n');

fprintf('\nAll figures saved to /figures/\n');