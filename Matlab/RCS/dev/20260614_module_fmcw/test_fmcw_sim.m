%% test_fmcw_sim.m - FMCW MIMO 雷达仿真完整测试套件
% 运行方式：在 MATLAB 命令窗口运行 >> test_fmcw_sim
%           或命令行: matlab -batch "run('test_fmcw_sim.m')"
%
% 测试项：
%   1. DBF 角度估计 + 静态双目标
%   2. FFT 角度估计
%   3. 杂波 + MTI 运动目标（Rayleigh 杂波）
%   4. Weibull 杂波 + MTI
%   5. 主入口 fmcw_sim_main 调用

clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), 'modules'));

pass_count = 0;
fail_count = 0;

%% ====== 测试 1：DBF 角度估计 + 静态双目标 ======
fprintf('\n====== 测试 1: DBF + 静态双目标 ======\n');
cfg1 = fmcw_sim_config('enable_MTI', false, 'enable_clutter', false, ...
    'SNR_dB', 30, 'az_angle_range', -60:2:60);
targets1 = [2.16, 0, 0, 10; 2.79, 0, -29.1, 10];

if_sig = fmcw_sim_signal(cfg1, targets1);
if_sig = fmcw_sim_noise(cfg1, if_sig);
[rf, cfg1] = fmcw_sim_range_fft(cfg1, if_sig);
[mti, cfg1] = fmcw_sim_mti(cfg1, rf);
[rd, cfg1] = fmcw_sim_doppler_fft(cfg1, mti);
[rd_v, ~, cfg1] = fmcw_sim_virtual_array(cfg1, rd);
[mask, ~, cfg1] = fmcw_sim_cfar(cfg1, rd_v);
[est1, cfg1] = fmcw_sim_angle(cfg1, rd_v, mask);
vr1 = fmcw_sim_verify(targets1, est1, cfg1);

fprintf('  Nv=%d, det=%d, pass=%d\n', cfg1.Nv_unique, length(est1), vr1.all_pass);
if vr1.all_pass, pass_count = pass_count + 1; else, fail_count = fail_count + 1; end

%% ====== 测试 2：FFT 角度估计 ======
fprintf('\n====== 测试 2: FFT 角度估计 ======\n');
cfg2 = fmcw_sim_config('enable_MTI', false, 'enable_clutter', false, ...
    'SNR_dB', 30, 'angle_method', 'FFT', 'az_angle_range', -60:2:60);
targets2 = [2.16, 0, 0, 10; 2.79, 0, -29.1, 10];

if_sig2 = fmcw_sim_signal(cfg2, targets2);
if_sig2 = fmcw_sim_noise(cfg2, if_sig2);
[rf2, cfg2] = fmcw_sim_range_fft(cfg2, if_sig2);
[mti2, cfg2] = fmcw_sim_mti(cfg2, rf2);
[rd2, cfg2] = fmcw_sim_doppler_fft(cfg2, mti2);
[rd_v2, ~, cfg2] = fmcw_sim_virtual_array(cfg2, rd2);
[mask2, ~, cfg2] = fmcw_sim_cfar(cfg2, rd_v2);
[est2, cfg2] = fmcw_sim_angle(cfg2, rd_v2, mask2);
vr2 = fmcw_sim_verify(targets2, est2, cfg2);

fprintf('  Nv=%d, det=%d, pass=%d\n', cfg2.Nv_unique, length(est2), vr2.all_pass);
if vr2.all_pass, pass_count = pass_count + 1; else, fail_count = fail_count + 1; end

%% ====== 测试 3：杂波 + MTI 运动目标 ======
fprintf('\n====== 测试 3: Rayleigh 杂波 + MTI 运动目标 ======\n');
cfg3 = fmcw_sim_config('enable_MTI', true, 'enable_clutter', true, ...
    'SNR_dB', 30, 'clutter_power', 1e-15, 'clutter_dist', 'Rayleigh', ...
    'az_angle_range', -60:2:60);
% 速度须在 Vmax_tdm 范围内且远离零速
targets3 = [2.16, 0.15, 0, 10; 2.79, 0.15, -29.1, 10];

if_sig3 = fmcw_sim_signal(cfg3, targets3);
if_sig3 = fmcw_sim_clutter(cfg3, if_sig3);
if_sig3 = fmcw_sim_noise(cfg3, if_sig3);
[rf3, cfg3] = fmcw_sim_range_fft(cfg3, if_sig3);
[mti3, cfg3] = fmcw_sim_mti(cfg3, rf3);
[rd3, cfg3] = fmcw_sim_doppler_fft(cfg3, mti3);
[rd_v3, ~, cfg3] = fmcw_sim_virtual_array(cfg3, rd3);
[mask3, ~, cfg3] = fmcw_sim_cfar(cfg3, rd_v3);
[est3, cfg3] = fmcw_sim_angle(cfg3, rd_v3, mask3);
vr3 = fmcw_sim_verify(targets3, est3, cfg3);

fprintf('  Nv=%d, det=%d, pass=%d\n', cfg3.Nv_unique, length(est3), vr3.all_pass);
if vr3.all_pass, pass_count = pass_count + 1; else, fail_count = fail_count + 1; end

%% ====== 测试 4：Weibull 杂波 + MTI ======
fprintf('\n====== 测试 4: Weibull 杂波 + MTI ======\n');
cfg4 = fmcw_sim_config('enable_MTI', true, 'enable_clutter', true, ...
    'SNR_dB', 30, 'clutter_power', 1e-15, 'clutter_dist', 'Weibull', ...
    'clutter_shape', 1.2, 'az_angle_range', -60:2:60);
targets4 = [2.16, 0.15, 0, 10; 2.79, 0.15, -29.1, 10];

if_sig4 = fmcw_sim_signal(cfg4, targets4);
if_sig4 = fmcw_sim_clutter(cfg4, if_sig4);
if_sig4 = fmcw_sim_noise(cfg4, if_sig4);
[rf4, cfg4] = fmcw_sim_range_fft(cfg4, if_sig4);
[mti4, cfg4] = fmcw_sim_mti(cfg4, rf4);
[rd4, cfg4] = fmcw_sim_doppler_fft(cfg4, mti4);
[rd_v4, ~, cfg4] = fmcw_sim_virtual_array(cfg4, rd4);
[mask4, ~, cfg4] = fmcw_sim_cfar(cfg4, rd_v4);
[est4, cfg4] = fmcw_sim_angle(cfg4, rd_v4, mask4);
vr4 = fmcw_sim_verify(targets4, est4, cfg4);

fprintf('  Nv=%d, det=%d, pass=%d\n', cfg4.Nv_unique, length(est4), vr4.all_pass);
if vr4.all_pass, pass_count = pass_count + 1; else, fail_count = fail_count + 1; end

%% ====== 测试 5：主入口调用 ======
fprintf('\n====== 测试 5: fmcw_sim_main 主入口 ======\n');
targets5 = [2.16, 0, 0, 10; 2.79, 0, -29.1, 10];
[est5, vr5, cfg5] = fmcw_sim_main(targets5, ...
    'enable_MTI', false, 'enable_clutter', false, ...
    'SNR_dB', 30, 'az_angle_range', -60:2:60, ...
    'enable_pointcloud', false);

fprintf('  Nv=%d, det=%d, pass=%d\n', cfg5.Nv_unique, length(est5), vr5.all_pass);
if vr5.all_pass, pass_count = pass_count + 1; else, fail_count = fail_count + 1; end

%% ====== 结果汇总 ======
fprintf('\n========================================\n');
fprintf('  测试结果汇总: %d/%d 通过\n', pass_count, pass_count + fail_count);
fprintf('========================================\n');

if fail_count > 0
    error('fmcw_sim_test:failed', '有 %d 项测试失败', fail_count);
end
