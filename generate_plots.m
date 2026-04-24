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

set(0, 'DefaultAxesFontSize',    12);
set(0, 'DefaultAxesFontName',    'Arial');
set(0, 'DefaultLineLineWidth',   1.5);
set(0, 'DefaultAxesBox',         'on');
set(0, 'DefaultFigureColor',     'w');

colors = [0.122 0.471 0.706;
          0.890 0.102 0.110;
          0.200 0.627 0.173];
seed_labels = {'Seed 111', 'Seed 222', 'Seed 333'};
seeds = [111, 222, 333];

function [t, d] = extractSignal(sim_out, name)
sig = sim_out.logsout.get(name);
t   = sig.Values.Time;
d   = sig.Values.Data;
if size(d, 2) > 1; d = d(:,1); end
end

% =========================================================
% TEST 1: THE BROADSIDE WALL
% =========================================================
fig1 = figure('Position', [100 100 900 500]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
ax1a = nexttile; hold on;
ax1b = nexttile; hold on;

for i = 1:3
    fname = sprintf('The_Broadside_Wall_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z]   = extractSignal(dummy_out, 'z');
    dir_sig  = dummy_out.yout.get('Pre_Wrap_Current_Direction');
    t_dir    = dir_sig.Values.Time;
    dir_data = rad2deg(dir_sig.Values.Data);

    plot(ax1a, t/3600, z,            'Color', colors(i,:), 'DisplayName', seed_labels{i});
    plot(ax1b, t_dir/3600, dir_data, 'Color', colors(i,:), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_worst')
        xline(ax1a, test_metrics.t_worst/3600, '--', 'Color', colors(i,:), 'LineWidth', 1, 'HandleVisibility', 'off');
        xline(ax1b, test_metrics.t_worst/3600, '--', 'Color', colors(i,:), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(ax1a, 5, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel(ax1a, 'Depth z (m)');
title(ax1a, 'Test 1: The Broadside Wall — Depth Response');
legend(ax1a, 'Location', 'southeast');
ylim(ax1a, [4.8 6.2]);
xlim(ax1a, [0 1]);
text(ax1a, 0.95, 4.83, 'Target: 5m', 'HorizontalAlignment','right', 'FontSize', 10);

ylabel(ax1b, 'Current Direction (deg)');
xlabel(ax1b, 'Time (hr)');
title(ax1b, 'Current Direction — accumulated (dashed = worst shift moment)');
xlim(ax1b, [0 1]);

saveas(fig1, 'figures/Test1_BroadsideWall.png');
fprintf('Saved Test 1\n');

% =========================================================
% TEST 2: THE EKMAN CORKSCREW
% =========================================================
fig2 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Ekman_Corkscrew_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i,:), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_complete') && ~isnan(test_metrics.t_complete)
        xline(test_metrics.t_complete, '--', 'Color', colors(i,:), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(50,        'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(50 + 0.98, 'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(50 - 0.98, 'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
text(400, 45, 'Target: 50m', 'HorizontalAlignment','left', 'FontSize', 10);
text(400, 42, '±2% band',    'HorizontalAlignment','left', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 2: The Ekman Corkscrew — Descent 1m \rightarrow 50m (dashed = settled)');
legend('Location', 'southeast');
xlim([0 600]);
ylim([0 60]);

saveas(fig2, 'figures/Test2_EkmanCorkscrew.png');
fprintf('Saved Test 2\n');

% =========================================================
% TEST 3: THE SILENT WINDUP
% =========================================================
fig3 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Silent_Windup_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i,:), 'DisplayName', seed_labels{i});
end

yline(30,        'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(30 + 0.096,'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(30 - 0.096,'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
text(300, 22, 'Target: 30m', 'HorizontalAlignment','left', 'FontSize', 10);
text(300, 19, 'RMS band',    'HorizontalAlignment','left', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 3: The Silent Windup — Calm Depth Hold at 30m');
legend('Location', 'southeast');
xlim([0 600]);
ylim([5 35]);

saveas(fig3, 'figures/Test3_SilentWindup.png');
fprintf('Saved Test 3\n');

% =========================================================
% TEST 4: THE SURFACE BREACH
% =========================================================
fig4 = figure('Position', [100 100 900 400]);
hold on;

for i = 1:3
    fname = sprintf('The_Surface_Breach_Seed_%d.mat', seeds(i));
    load(fname, 'dummy_out', 'test_metrics');
    [t, z] = extractSignal(dummy_out, 'z');
    plot(t, z, 'Color', colors(i,:), 'DisplayName', seed_labels{i});
end

yline(5,       'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(5 + 1.9, 'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(5 - 1.9, 'k:',  'LineWidth', 1.0, 'HandleVisibility', 'off');
text(400, 25, 'Target: 5m', 'HorizontalAlignment','left', 'FontSize', 10);
text(400, 20, '±2% band',   'HorizontalAlignment','left', 'FontSize', 9, 'Color', [0.4 0.4 0.4]);

ylabel('Depth z (m)');
xlabel('Time (s)');
title('Test 4: The Surface Breach — Ascent 100m \rightarrow 5m (Stormy)');
legend('Location', 'northeast');
xlim([0 600]);
ylim([-20 120]);

saveas(fig4, 'figures/Test4_SurfaceBreach.png');
fprintf('Saved Test 4\n');

% =========================================================
% TEST 5: THE OPERATIONAL ENVELOPE
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

    plot(ax5a, t, yaw, 'Color', colors(i,:), 'DisplayName', seed_labels{i});
    plot(ax5b, t, z,   'Color', colors(i,:), 'DisplayName', seed_labels{i});

    if isfield(test_metrics, 't_heading_settle') && ~isnan(test_metrics.t_heading_settle)
        xline(ax5a, test_metrics.t_heading_settle, '--', 'Color', colors(i,:), 'LineWidth', 1, 'HandleVisibility', 'off');
    end
end

yline(ax5a, -60, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax5a, 3900, -57, 'Target: -60 rad', 'HorizontalAlignment','right', 'FontSize', 10);
ylabel(ax5a, 'Yaw \psi (rad)');
title(ax5a, 'Test 5: Operational Envelope — Heading 0 \rightarrow -60 rad');
legend(ax5a, 'Location', 'southeast');
xlim(ax5a, [0 4000]);
ylim(ax5a, [-65 5]);

yline(ax5b, 10, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax5b, 3900, 10.3, 'Target: 10m', 'HorizontalAlignment','right', 'FontSize', 10);
ylabel(ax5b, 'Depth z (m)');
xlabel(ax5b, 'Time (s)');
title(ax5b, 'Depth Coupling During Heading Maneuver');
xlim(ax5b, [0 4000]);
ylim(ax5b, [0 12]);

saveas(fig5, 'figures/Test5_OperationalEnvelope.png');
fprintf('Saved Test 5\n');

% =========================================================
% TEST 6: THE DEEP FREEZE
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

yline(ax6a, -60,  'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax6a, 3900, -57, 'Target: -60 rad', 'HorizontalAlignment','right', 'FontSize', 10);
ylabel(ax6a, 'Yaw \psi (rad)');
title(ax6a, 'Test 6: The Deep Freeze — Heading 0 \rightarrow -60 rad (Stormy, 100m)');
legend(ax6a, 'Location', 'southeast');
xlim(ax6a, [0 4000]);
ylim(ax6a, [-65 5]);

yline(ax6b, 100, 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
text(ax6b, 3900, 100.8, 'Target: 100m', 'HorizontalAlignment','right', 'FontSize', 10);
ylabel(ax6b, 'Depth z (m)');
xlabel(ax6b, 'Time (s)');
title(ax6b, 'Depth Hold Under Stormy Conditions + Heading Maneuver');
xlim(ax6b, [0 4000]);
ylim(ax6b, [55 115]);

saveas(fig6, 'figures/Test6_DeepFreeze.png');
fprintf('Saved Test 6\n');

fprintf('\nAll figures saved to /figures/\n');