%% test_two_targets.m - 双目标测试（含俯仰角，验证4元俯仰虚拟阵列）
% 运行方式：matlab -batch "run('test_two_targets.m')"
%
% 目标：
%   目标1: R=5m, v=0, Az=10°, El=5°
%   目标2: R=10m, v=0, Az=-15°, El=0°

clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), 'modules'));

%% ====== 目标定义 ======
% 格式: [R(m), v(m/s), θ_az(°), θ_el(°), RCS(m²)]
targets = [
    5.0,   0,   10.0,   5.0,  10;
   10.0,   0,  -15.0,   0.0,  20;
];

fprintf('====== 双目标测试（含俯仰角）======\n');
fprintf('目标1: R=%.1fm, v=%.1fm/s, Az=%.1f°, El=%.1f°, RCS=%.0fm²\n', ...
    targets(1,1), targets(1,2), targets(1,3), targets(1,4), targets(1,5));
fprintf('目标2: R=%.1fm, v=%.1fm/s, Az=%.1f°, El=%.1f°, RCS=%.0fm²\n', ...
    targets(2,1), targets(2,2), targets(2,3), targets(2,4), targets(2,5));

%% ====== 仿真运行（关闭MTI，静态目标）======
[estimates, verify_result, cfg] = fmcw_sim_main(targets, ...
    'enable_MTI',       false, ...
    'enable_clutter',   false, ...
    'SNR_dB',           30, ...
    'angle_method',     'DBF', ...
    'az_angle_range',   -60:0.5:60, ...
    'el_angle_range',   -30:0.5:30, ...
    'enable_pointcloud', false, ...
    'save_fig',          false);

%% ====== 俯仰角精度分析 ======
fprintf('\n====== 俯仰角估计精度分析 ======\n');
fprintf('4元俯仰阵列位置: [0, 0.5, 2, 3] lambda\n');
fprintf('阵列孔径: 3 lambda, 理论3dB波束宽度: %.1f deg\n', ...
    rad2deg(0.886 * cfg.lambda / (3 * cfg.lambda)));

for t = 1:size(targets, 1)
    true_el = targets(t, 4);
    if verify_result.matched(t)
        eidx = verify_result.match_idx(t);
        est_el = estimates(eidx).elevation;
        err_el = est_el - true_el;
        fprintf('目标%d: 真值El=%.1f deg, 估计El=%.2f deg, 误差=%.2f deg\n', ...
            t, true_el, est_el, err_el);
    else
        fprintf('目标%d: 未匹配\n', t);
    end
end

%% ====== 测试结论 ======
fprintf('\n====== 测试结论 ======\n');
if verify_result.all_pass
    fprintf('双目标测试: 通过\n');
else
    fprintf('双目标测试: 失败\n');
    for t = 1:size(targets, 1)
        if ~verify_result.target_pass(t)
            fprintf('  目标%d 未通过\n', t);
        end
    end
end
