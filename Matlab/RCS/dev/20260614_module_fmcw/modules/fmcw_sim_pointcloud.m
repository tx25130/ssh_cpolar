function [pointcloud, fig_handles] = fmcw_sim_pointcloud(cfg, estimates, targets)
% fmcw_sim_pointcloud  点云结构化输出与可视化（含俯仰角）
%
%   [pointcloud, fig_handles] = fmcw_sim_pointcloud(cfg, estimates, targets)
%
%   输入：
%     cfg       - fmcw_sim_config() 返回的参数结构体
%     estimates - 角度估计结果结构体数组（含 elevation 字段）
%     targets   - 目标真值矩阵 [N×4] 或 [N×5]（可选，用于对比显示）
%
%   输出：
%     pointcloud   - 结构体数组，每点含 range, velocity, angle, elevation, snr_dB, x, y, z
%     fig_handles  - 图形句柄数组

    %% ====== 结构化输出 ======
    num_det = length(estimates);
    pointcloud = struct('range', cell(1, num_det), ...
                        'velocity', cell(1, num_det), ...
                        'angle', cell(1, num_det), ...
                        'elevation', cell(1, num_det), ...
                        'snr_dB', cell(1, num_det), ...
                        'power', cell(1, num_det), ...
                        'x', cell(1, num_det), ...
                        'y', cell(1, num_det), ...
                        'z', cell(1, num_det));

    for i = 1:num_det
        R = estimates(i).range;
        V = estimates(i).velocity;
        theta_az = estimates(i).angle;
        theta_el = estimates(i).elevation;
        snr = estimates(i).snr_dB;

        % 笛卡尔坐标转换（含俯仰角 → 3D坐标）
        az_rad = deg2rad(theta_az);
        el_rad = deg2rad(theta_el);
        x = R * cos(el_rad) * sin(az_rad);
        y = R * cos(el_rad) * cos(az_rad);
        z = R * sin(el_rad);

        power = 10^(snr / 10);

        pointcloud(i).range     = R;
        pointcloud(i).velocity  = V;
        pointcloud(i).angle     = theta_az;
        pointcloud(i).elevation = theta_el;
        pointcloud(i).snr_dB    = snr;
        pointcloud(i).power     = power;
        pointcloud(i).x         = x;
        pointcloud(i).y         = y;
        pointcloud(i).z         = z;
    end

    %% ====== 可视化 ======
    fig_handles = [];

    % 提取数据用于绘图
    if num_det > 0
        det_R     = [pointcloud.range];
        det_V     = [pointcloud.velocity];
        det_theta = [pointcloud.angle];
        det_el    = [pointcloud.elevation];
        det_x     = [pointcloud.x];
        det_y     = [pointcloud.y];
        det_z     = [pointcloud.z];
        det_snr   = [pointcloud.snr_dB];
    end

    % 真值数据
    has_targets = (nargin >= 3) && ~isempty(targets);
    if has_targets
        tgt_R  = targets(:, 1);
        tgt_V  = targets(:, 2);
        tgt_az = targets(:, 3);
        if size(targets, 2) >= 5
            tgt_el = targets(:, 4);
        else
            tgt_el = zeros(size(targets, 1), 1);
        end
        tgt_x = tgt_R .* cosd(tgt_el) .* sind(tgt_az);
        tgt_y = tgt_R .* cosd(tgt_el) .* cosd(tgt_az);
        tgt_z = tgt_R .* sind(tgt_el);
    end

    %% ====== 图1：距离-多普勒图 ======
    fig1 = figure('Name', '距离-多普勒图', 'NumberTitle', 'off');
    if isfield(cfg, 'V_axis_tdm')
        V_axis = cfg.V_axis_tdm;
    else
        V_axis = cfg.V_axis;
    end

    if num_det > 0
        scatter(det_R, det_V, 50, det_snr, 'filled', 'MarkerEdgeColor', 'w');
        colorbar;
        colormap(jet);
        caxis([min(det_snr)-5, max(det_snr)+5]);
    end

    if has_targets
        hold on;
        plot(tgt_R, tgt_V, 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
        hold off;
    end

    xlabel('距离 (m)');
    ylabel('速度 (m/s)');
    title('距离-多普勒图');
    grid on;
    fig_handles(end+1) = fig1;

    %% ====== 图2：BEV 鸟瞰图 ======
    fig2 = figure('Name', 'BEV 鸟瞰图', 'NumberTitle', 'off');
    hold on;

    max_R = max(cfg.Rmax, 50);
    for r = 10:10:max_R
        theta_circle = linspace(0, 2*pi, 100);
        plot(r*cos(theta_circle), r*sin(theta_circle), 'k--', 'LineWidth', 0.5);
        text(r*0.7, r*0.7, sprintf('%dm', r), 'FontSize', 8, 'Color', [0.5 0.5 0.5]);
    end

    fov = 120;
    fov_rad = deg2rad(fov/2);
    max_line = max_R;
    plot([0, max_line*sin(fov_rad)], [0, max_line*cos(fov_rad)], 'k-', 'LineWidth', 0.5);
    plot([0, -max_line*sin(fov_rad)], [0, max_line*cos(fov_rad)], 'k-', 'LineWidth', 0.5);

    rectangle('Position', [-0.5, -1, 1, 2], 'FaceColor', 'g', 'EdgeColor', 'k');

    if num_det > 0
        scatter(det_x, det_y, 40, det_snr, 'filled');
        colormap(jet);
        colorbar;
    end

    if has_targets
        plot(tgt_x, tgt_y, 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    end

    hold off;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('BEV 鸟瞰图');
    grid on;
    fig_handles(end+1) = fig2;

    %% ====== 图3：3D 点云（含俯仰角） ======
    if num_det > 0
        fig3 = figure('Name', '3D 点云', 'NumberTitle', 'off');
        scatter3(det_x, det_y, det_z, 40, det_snr, 'filled');
        colormap(jet);
        colorbar;
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('3D 点云（颜色=SNR）');
        grid on;
        if has_targets
            hold on;
            scatter3(tgt_x, tgt_y, tgt_z, 80, 'm^', 'filled');
            hold off;
        end
        fig_handles(end+1) = fig3;
    end
end
