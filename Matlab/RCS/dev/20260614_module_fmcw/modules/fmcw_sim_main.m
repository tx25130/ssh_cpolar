function [estimates, verify_result, cfg] = fmcw_sim_main(targets, varargin)
% fmcw_sim_main  FMCW MIMO 雷达仿真主入口
%
%   [estimates, verify_result, cfg] = fmcw_sim_main(targets)
%   [estimates, verify_result, cfg] = fmcw_sim_main(targets, 'fc', 79e9, ...)
%
%   输入：
%     targets  - 目标矩阵 [N×4] 或 [N×5]
%                [R(m), v(m/s), θ_az(°), RCS(m²)] 或
%                [R(m), v(m/s), θ_az(°), θ_el(°), RCS(m²)]
%     varargin - 名称-值对参数覆盖（传入 fmcw_sim_config）
%
%   输出：
%     estimates      - 检测估计结果结构体数组
%     verify_result  - 验证结果结构体
%     cfg            - 完整配置结构体

    %% ====== 步骤 1/12：参数配置 ======
    fprintf('[步骤 1/12] 参数配置...\n');
    cfg = fmcw_sim_config(varargin{:});

    fprintf('  载频: %.1f GHz, 波长: %.4f mm\n', cfg.fc/1e9, cfg.lambda*1e3);
    fprintf('  距离分辨率: %.4f m, 速度分辨率: %.4f m/s\n', cfg.Rres, cfg.Vres);
    fprintf('  最大距离: %.2f m, 最大速度: %.2f m/s\n', cfg.Rmax, cfg.Vmax);
    fprintf('  方位TX=%d, 俯仰TX=%d, 总TX=%d, RX=%d\n', ...
        cfg.Ntx, cfg.Ntx_el, cfg.Ntx_total, cfg.Nrx);
    fprintf('  方位虚拟阵元: %d (去重叠 %d)\n', cfg.Nv, cfg.Nv_unique);
    fprintf('  俯仰虚拟阵元: %d (去重叠 %d)\n', cfg.Nv_el, cfg.Nv_el_unique);
    fprintf('  角度估计方法: %s\n', cfg.angle_method);
    fprintf('  杂波: %s, MTI: %s\n', ...
        mat2str(cfg.enable_clutter), mat2str(cfg.enable_MTI));
    fprintf('参数配置完成\n\n');

    %% ====== 步骤 2/12：信号生成 ======
    fprintf('[步骤 2/12] TDM-MIMO 差频信号生成...\n');
    if_signal = fmcw_sim_signal(cfg, targets);
    fprintf('  信号维度: [%s]\n', num2str(size(if_signal)));
    fprintf('  信号功率: %.2e W\n', mean(abs(if_signal(:)).^2));
    fprintf('信号生成完成\n\n');

    %% ====== 步骤 3/12：杂波叠加 ======
    fprintf('[步骤 3/12] 杂波叠加...\n');
    if_signal = fmcw_sim_clutter(cfg, if_signal);
    if cfg.enable_clutter
        fprintf('  杂波分布: %s, 功率: %.2e W\n', ...
            cfg.clutter_dist, cfg.clutter_power);
    else
        fprintf('  杂波已禁用\n');
    end
    fprintf('杂波叠加完成\n\n');

    %% ====== 步骤 4/12：噪声叠加 ======
    fprintf('[步骤 4/12] 噪声叠加...\n');
    if_signal = fmcw_sim_noise(cfg, if_signal);
    fprintf('  SNR: %d dB\n', cfg.SNR_dB);
    fprintf('噪声叠加完成\n\n');

    %% ====== 步骤 5/12：Range-FFT ======
    fprintf('[步骤 5/12] Range-FFT 测距...\n');
    [range_fft_data, cfg] = fmcw_sim_range_fft(cfg, if_signal);
    fprintf('  距离分辨率 = %.4f m\n', cfg.Rres);
    fprintf('Range-FFT 完成\n\n');

    %% ====== 步骤 6/12：MTI 对消 ======
    fprintf('[步骤 6/12] MTI 对消...\n');
    [mti_data, cfg] = fmcw_sim_mti(cfg, range_fft_data);
    if cfg.enable_MTI
        fprintf('  对消后慢时间长度: %d (原 %d)\n', ...
            size(mti_data, 2), size(range_fft_data, 2));
    else
        fprintf('  MTI 已禁用\n');
    end
    fprintf('MTI 对消完成\n\n');

    %% ====== 步骤 7/12：Doppler-FFT ======
    fprintf('[步骤 7/12] Doppler-FFT 测速...\n');
    [rd_data, cfg] = fmcw_sim_doppler_fft(cfg, mti_data);
    fprintf('  TDM 速度分辨率 = %.4f m/s\n', cfg.Vres_tdm);
    fprintf('Doppler-FFT 完成\n\n');

    %% ====== 步骤 8/12：虚拟阵列重组 ======
    fprintf('[步骤 8/12] MIMO 虚拟阵列重组（方位 + 俯仰）...\n');
    [rd_virtual, rd_el_virtual, cfg] = fmcw_sim_virtual_array(cfg, rd_data);
    fprintf('  方位虚拟阵元: %d (去重叠后 %d)\n', cfg.Nv, cfg.Nv_unique);
    fprintf('  俯仰虚拟阵元: %d (去重叠后 %d)\n', cfg.Nv_el, cfg.Nv_el_unique);
    fprintf('虚拟阵列重组完成\n\n');

    %% ====== 步骤 9/12：CFAR 检测 ======
    fprintf('[步骤 9/12] 二维 CA-CFAR 检测...\n');
    [det_mask, det_snr, cfg] = fmcw_sim_cfar(cfg, rd_virtual);
    num_det = sum(det_mask(:));
    fprintf('  检测点数: %d\n', num_det);
    fprintf('  门限系数 alpha = %.2f\n', cfg.cfar_alpha);
    fprintf('CFAR 检测完成\n\n');

    %% ====== 步骤 10/12：角度估计 ======
    fprintf('[步骤 10/12] 角度估计 (%s, 含俯仰)...\n', cfg.angle_method);
    [estimates, cfg] = fmcw_sim_angle(cfg, rd_virtual, det_mask, rd_el_virtual);
    fprintf('  估计目标数: %d\n', length(estimates));
    for i = 1:length(estimates)
        fprintf('    目标%d: R=%.3fm, V=%.3fm/s, Az=%.2f°, El=%.2f°, SNR=%.1fdB\n', ...
            i, estimates(i).range, estimates(i).velocity, ...
            estimates(i).angle, estimates(i).elevation, estimates(i).snr_dB);
    end
    fprintf('角度估计完成\n\n');

    %% ====== 步骤 11/12：点云可视化 ======
    if cfg.enable_pointcloud
        fprintf('[步骤 11/12] 点云可视化...\n');
        [~, fig_handles] = fmcw_sim_pointcloud(cfg, estimates, targets);
        fprintf('点云可视化完成\n\n');
    else
        fig_handles = [];
        fprintf('[步骤 11/12] 点云可视化已跳过\n\n');
    end

    %% ====== 步骤 12/12：仿真验证 ======
    fprintf('[步骤 12/12] 仿真验证...\n');
    verify_result = fmcw_sim_verify(targets, estimates, cfg);
    fprintf('仿真验证完成\n\n');

    %% ====== 自动保存所有图像 ======
    if isfield(cfg, 'save_fig') && cfg.save_fig
        save_dir = cfg.fig_dir;
        if isempty(save_dir)
            save_dir = pwd;
        end
        if ~exist(save_dir, 'dir')
            mkdir(save_dir);
        end

        fig_all = flip(findall(0, 'Type', 'figure'));
        if isempty(fig_all)
            fprintf('无打开的图形窗口，跳过图像保存\n\n');
        else
            fprintf('保存 %d 张图像到: %s\n', length(fig_all), save_dir);
            fig_names = cell(1, length(fig_all));
            for i = 1:length(fig_all)
                name = get(fig_all(i), 'Name');
                if isempty(name)
                    name = sprintf('figure_%d', i);
                end
                name = strrep(name, ' ', '_');
                name = strrep(name, '-', '_');
                fig_names{i} = name;
            end
            warning('off', 'MATLAB:handle_graphics:exceptions:SceneNode');
            for i = 1:length(fig_all)
                fh = fig_all(i);
                if ~isvalid(fh)
                    continue;
                end
                drawnow;
                fpath = fullfile(save_dir, sprintf('%02d_%s.png', i, fig_names{i}));
                print(fh, fpath, '-dpng', '-r150');
                fprintf('  [%d/%d] %s\n', i, length(fig_all), fpath);
            end
            warning('on', 'MATLAB:handle_graphics:exceptions:SceneNode');
            fprintf('图像保存完成\n\n');
        end
    end

    %% ====== 结果汇总 ======
    fprintf('============ 结果汇总 ============\n');
    fprintf('MIMO 配置: %dAz-Tx × %dRx = %d方位阵元 (%d去重叠) + %dEl-Tx = %d俯仰阵元 (%d去重叠)\n', ...
        cfg.Ntx, cfg.Nrx, cfg.Nv, cfg.Nv_unique, cfg.Ntx_el, cfg.Nv_el, cfg.Nv_el_unique);
    fprintf('目标数: %d, 检测点数: %d\n', size(targets,1), num_det);
    fprintf('角度算法: %s\n', cfg.angle_method);
    fprintf('杂波: %s, MTI: %s\n', ...
        mat2str(cfg.enable_clutter), mat2str(cfg.enable_MTI));

    % 真值 vs 估计对比表
    has_el = size(targets, 2) >= 5;
    fprintf('\n--- 真值 vs 估计 ---\n');
    if has_el
        fprintf('%-6s  %-32s  %-32s  %-8s\n', '目标', '真值 [R,V,Az,El,RCS]', '估计 [R,V,Az,El,SNR]', '状态');
    else
        fprintf('%-6s  %-24s  %-24s  %-8s\n', '目标', '真值 [R,V,Az,RCS]', '估计 [R,V,Az,SNR]', '状态');
    end
    fprintf('%s\n', repmat('-', 1, 80));
    for t = 1:size(targets, 1)
        if has_el
            tgt_str = sprintf('[%.2f, %.2f, %.1f, %.1f, %.1f]', ...
                targets(t,1), targets(t,2), targets(t,3), targets(t,4), targets(t,5));
        else
            tgt_str = sprintf('[%.2f, %.2f, %.1f, %.1f]', ...
                targets(t,1), targets(t,2), targets(t,3), targets(t,4));
        end
        if verify_result.matched(t)
            eidx = verify_result.match_idx(t);
            est = estimates(eidx);
            est_str = sprintf('[%.3f, %.3f, %.2f, %.2f, %.1f]', ...
                est.range, est.velocity, est.angle, est.elevation, est.snr_dB);
            if verify_result.target_pass(t)
                status_str = 'PASS';
            else
                status_str = 'FAIL';
            end
        else
            est_str = '--- 未匹配 ---';
            status_str = 'FAIL';
        end
        fprintf('  %-4d  %-32s  %-32s  %-8s\n', t, tgt_str, est_str, status_str);
    end

    % RMSE 统计
    if ~isnan(verify_result.rmse_range)
        fprintf('\nRMSE: R=%.4f m, V=%.4f m/s, Az=%.3f°, El=%.3f°\n', ...
            verify_result.rmse_range, verify_result.rmse_velocity, ...
            verify_result.rmse_angle, verify_result.rmse_elevation);
    end

    % 估计结果详情
    if num_det > 0
        fprintf('\n--- 估计结果详情 ---\n');
        for i = 1:length(estimates)
            fprintf('  检测点%d: R=%.4f m, V=%.4f m/s, Az=%.2f°, El=%.2f°, SNR=%.1f dB\n', ...
                i, estimates(i).range, estimates(i).velocity, ...
                estimates(i).angle, estimates(i).elevation, estimates(i).snr_dB);
        end
    end

    if verify_result.all_pass
        fprintf('\n验证结果: 通过\n');
    else
        fprintf('\n验证结果: 失败\n');
    end
    fprintf('===================================\n');
end
