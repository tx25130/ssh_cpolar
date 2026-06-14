%% test_timing.m - 各模块计时测试
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), 'modules'));

cfg = fmcw_sim_config('enable_MTI', false, 'enable_clutter', false, ...
    'SNR_dB', 30, 'az_angle_range', -60:2:60);
targets = [2.16, 0, 0, 10; 2.79, 0, -29.1, 10];

t_sig = tic; if_sig = fmcw_sim_signal(cfg, targets); t_sig = toc(t_sig);
t_noi = tic; if_sig = fmcw_sim_noise(cfg, if_sig); t_noi = toc(t_noi);
t_rff = tic; [rf, cfg] = fmcw_sim_range_fft(cfg, if_sig); t_rff = toc(t_rff);
t_mti = tic; [mti, cfg] = fmcw_sim_mti(cfg, rf); t_mti = toc(t_mti);
t_dop = tic; [rd, cfg] = fmcw_sim_doppler_fft(cfg, mti); t_dop = toc(t_dop);
t_va  = tic; [rd_v, ~, cfg] = fmcw_sim_virtual_array(cfg, rd); t_va = toc(t_va);
t_cfa = tic; [mask, ~, cfg] = fmcw_sim_cfar(cfg, rd_v); t_cfa = toc(t_cfa);
t_ang = tic; [est, cfg] = fmcw_sim_angle(cfg, rd_v, mask); t_ang = toc(t_ang);
t_ver = tic; vr = fmcw_sim_verify(targets, est, cfg); t_ver = toc(t_ver);

fprintf('\n=== 各模块耗时 ===\n');
fprintf('信号生成:  %.3f s\n', t_sig);
fprintf('噪声:      %.3f s\n', t_noi);
fprintf('Range-FFT: %.3f s\n', t_rff);
fprintf('MTI:       %.3f s\n', t_mti);
fprintf('Doppler:   %.3f s\n', t_dop);
fprintf('虚拟阵列:  %.3f s\n', t_va);
fprintf('CFAR:      %.3f s\n', t_cfa);
fprintf('角度估计:  %.3f s\n', t_ang);
fprintf('验证:      %.3f s\n', t_ver);
fprintf('总计:      %.3f s\n', t_sig+t_noi+t_rff+t_mti+t_dop+t_va+t_cfa+t_ang+t_ver);

fprintf('\n=== 检测结果 ===\n');
fprintf('Nv_unique=%d, 检测数=%d, all_pass=%d\n', cfg.Nv_unique, length(est), vr.all_pass);
for i = 1:length(est)
    fprintf('det%d: R=%.3f V=%.3f θ=%.2f\n', i, est(i).range, est(i).velocity, est(i).angle);
end
