%% spm_full_model.m
% =========================================================================
% 复现论文: "Road-Condition Recognition Using 24-GHz Automotive Radar"
%   Ville V. Viikari, Timo Varpula, and Mikko Kantanen
%   IEEE Trans. Intell. Transp. Syst., vol. 10, no. 4, pp. 789-797, 2009
%
% 功能: 严格依据论文 Section IV-A 的小扰动模型 (Small-Perturbation
%       Model) 公式 (9)-(18)，计算湿润/干燥沥青表面的后向散射系数
%       σ⁰_vv 和 σ⁰_hh，复现论文 Figure 12 的理论曲线。
%
% 实现说明:
%   论文公式 (9) 中的 |I^n_pp| 按量纲分析应为 |I^n_pp|²（散射截面
%   为功率比量），这与标准 SPM (Fung 1994) 一致。OCR 文本中的
%   绝对值符号可能丢失了平方上标。
%
%   公式 (10) 中的 exp(-k²_z·γ²) 因子在标准 Fung SPM 中位于求和
%   号外侧的 exp(-2k²_z·γ²) 中统一处理；此处按论文原文保留，
%   同时提供不含该因子的对比模式。
%
%   公式 (14) 的 F_hh 项在 μ_r=1 的非磁性高介电常数表面上，
%   第二项含 (ε_r-1)/cos²φ，在大入射角 (φ>70°) 发散。
%   这与标准 SPM 中 HH 极化没有奇点的事实不符。
%   OCR 可能丢失了 ε_r 因子。本程序默认不计入 F_pp 修正项，
%   仅使用 Bragg 共振项 (f_pp)，与标准一阶 SPM 一致。
%   可通过设置 use_F_terms = true 启用完整 F_pp 项以观察发散效应。
%
% 公式来源:
%   论文公式 (9)-(18)，原始来源:
%   [21] A. K. Fung, Microwave Scattering and Emission Models and
%        Their Applications. Boston: Artech House, 1994.
%
% 环境要求: MATLAB R2020a+, Signal Processing Toolbox
% =========================================================================
clc; clear; close all;

%% ========================================================================
%  1. 物理常数与雷达系统参数
% =========================================================================
c0 = 3e8;                % 真空中光速 (m/s)
freq = 24e9;             % 雷达工作频率 24 GHz (论文 Section V, Table II)
lambda = c0 / freq;      % 波长 λ = c/f = 12.5 mm
k = 2 * pi / lambda;     % 自由空间波数 k = 2π/λ (rad/m)

fprintf('╔══════════════════════════════════════════╗\n');
fprintf('║  SPM 后向散射模型 — Viikari et al. 2009 ║\n');
fprintf('╚══════════════════════════════════════════╝\n\n');
fprintf('雷达系统参数:\n');
fprintf('  频率 f  = %.1f GHz\n', freq/1e9);
fprintf('  波长 λ  = %.2f mm\n', lambda*1e3);
fprintf('  波数 k  = %.2f rad/m\n\n', k);

%% ========================================================================
%  2. 沥青表面粗糙度参数 (论文 Table I)
%
%     双尺度高斯相关粗糙面模型:
%     - 大尺度 (γ₁, L₁): 宏观起伏
%     - 小尺度 (γ₂, L₂): Bragg 共振散射源
%     - 两个尺度互不相关 (uncorrelated)
%
%     参数来源 (论文 IV-B 节, p.795):
%     "As asphalt surface statistics ... are not directly available;
%      the model is fitted to the laboratory measurement results."
%     拟合目标函数 (论文 Eq.20): 最小化模型与实测 σ⁰(dB) 的
%     L1 范数积分。
%
%     【参数选择依据】
%     - 全部来自论文 Table I，是拟合实验室实测数据得到的最佳值
%     - γ: RMS 表面高度 (root-mean-square height)，论文原文表述
%       为 "RMS height of the surface slope" 但实际指表面高度
%       (量纲 mm，与典型 RMS 高度一致)
%     - L: 表面自相关长度 (correlation length)
% =========================================================================
gamma1_mm = 0.6545;      % 大尺度 RMS 高度 (mm) — 论文 Table I
L1_mm     = 14.1840;     % 大尺度自相关长度 (mm)
gamma2_mm = 0.2018;      % 小尺度 RMS 高度 (mm)
L2_mm     = 3.7632;      % 小尺度自相关长度 (mm)

% 转换为国际单位制 (m) — MATLAB 内部统一用 SI
gamma1 = gamma1_mm * 1e-3;
L1     = L1_mm     * 1e-3;
gamma2 = gamma2_mm * 1e-3;
L2     = L2_mm     * 1e-3;

% 总方差与总 RMS 高度
%   论文: γ² = γ²₁ + γ²₂ (total variance of the slope height)
gamma_sq = gamma1^2 + gamma2^2;  % 总高度方差 γ² (m²)
gamma    = sqrt(gamma_sq);       % 总 RMS 高度 γ (m)

% 双尺度归一化权重 (论文 Eq.17 注释)
%   原文: a = γ²₁/γ, b = γ²₂/γ
%   【修正依据】量纲分析: a,b 应为无量纲权重且 a+b=1。
%   OCR 疑丢失 γ 的平方上标 (/γ → /γ²)。此处采用标准方差占比:
%     a = γ²₁/γ², b = γ²₂/γ²  (且 a+b=1)
%   此修正在标准微波遥感教科书中为通行做法 (Fung 1994, Ch.6)。
a_weight = gamma1^2 / gamma_sq;
b_weight = gamma2^2 / gamma_sq;

fprintf('表面粗糙度参数 (论文 Table I):\n');
fprintf('  大尺度: γ₁ = %.4f mm,  L₁ = %.4f mm\n', gamma1_mm, L1_mm);
fprintf('  小尺度: γ₂ = %.4f mm,  L₂ = %.4f mm\n', gamma2_mm, L2_mm);
fprintf('  总 RMS 高度 γ = %.4f mm\n', gamma*1e3);
fprintf('  大尺度权重 a = γ²₁/γ² = %.4f\n', a_weight);
fprintf('  小尺度权重 b = γ²₂/γ² = %.4f\n', b_weight);
fprintf('  k·γ = %.3f  (SPM 有效性参数)\n\n', k*gamma);

%% ========================================================================
%  3. 材料电磁参数
%
%     湿润沥青 ε_r = 33 (论文 IV-B 节, p.795)
%       【选择依据】论文明确说明 (p.795 右栏):
%       "According to the model presented in [24], the relative
%        permittivity of 20°C water is 33-j36 at 24 GHz, resulting
%        into a skin depth of 0.77 mm. Because the water-layer
%        thickness in the experiments was substantially higher than
%        the skin depth, the relative permittivity of wet asphalt
%        can be taken to be the same as that of the water. In this
%        model, the relative permittivity of water is set to 33."
%       参考: Meissner & Wentz, IEEE TGRS, 2004 [24]
%
%     干燥沥青 ε_r = 2.6 (论文 IV-B 节, p.795)
%       【选择依据】论文引用 [22][23]:
%       "the dielectric constant of asphalt is 2.6 at microwave
%        frequencies. This value is used in the model for dry asphalt."
%
%     磁导率 μ_r = 1
%       【选择依据】所有表面均为非磁性材料，标准假设。
%       论文未明确说明但所有公式均适用 μ_r=1。
%
%     【虚部忽略说明】水在 24 GHz 的复介电常数为 33-j36。
%     标准 SPM 假设无耗或低耗介质。论文仅使用实部 ε_r=33，
%     因为趋肤深度 (0.77 mm) 远小于水层厚度，损耗主要影响
%     透射波衰减，对表面 Bragg 散射影响有限。
%     (Fung 1994, Ch.2 的标准 SPM 也仅使用实部)
% =========================================================================
epsilon_r_wet = 33;      % 湿润沥青: 论文 IV-B 节 + 参考[24]
epsilon_r_dry = 2.6;     % 干燥沥青: 论文 IV-B 节 + 参考[22][23]
mu_r = 1;                % 相对磁导率: 非磁性材料

fprintf('材料电磁参数:\n');
fprintf('  湿润沥青 ε_r = %d,    μ_r = %d\n', epsilon_r_wet, mu_r);
fprintf('  干燥沥青 ε_r = %.1f,  μ_r = %d\n', epsilon_r_dry, mu_r);
fprintf('  注: 水的虚部 j36 (趋肤深度 0.77 mm) 未计入\n\n');

%% ========================================================================
%  4. 入射角范围
%     论文 Figure 12 覆盖 0°-80° (实验室测量步进 5°)
%     此处使用 0.5° 步进以保证曲线平滑和关键角度的精确取值
% =========================================================================
phi_deg = (0:0.5:80).';   % 入射角 (deg)，相对于表面法线
phi = deg2rad(phi_deg);   % 入射角 (rad)

%% ========================================================================
%  5. 控制标志
%     use_F_terms: 是否包含论文 Eq.(13-14) 的 F_pp 修正项
%       = false (默认): 仅使用 Bragg 共振项 (f_pp)，与标准 SPM 一致
%       = true: 使用完整 Eq.(10)，包含 F_pp 项
%       【注意】启用 F_pp 后 HH 极化在 φ>65° 处发散 (原因见文件头注释)
%     use_exp_in_I: 是否保留 Eq.(10) 中的 exp(-k²_z·γ²)
%       = false (默认): 不计入，与标准 Fung SPM 一致
%       = true: 按 OCR 文本计入
% =========================================================================
use_F_terms = false;      % 是否启用 F_pp 修正项
use_exp_in_I = false;     % I^n 中是否保留额外指数衰减

fprintf('模型设置:\n');
fprintf('  F_pp 修正项: %s\n', ternary_str(use_F_terms, '启用', '禁用'));
fprintf('  I^n 中 exp 因子: %s\n', ternary_str(use_exp_in_I, '保留', '移除'));
fprintf('  级数截断: n_max=20, tol=1e-8\n\n');

%% ========================================================================
%  6. 计算后向散射系数
% =========================================================================
fprintf('正在计算...\n');

% --- 湿润沥青 (主要目标，对应论文 Figure 12) ---
[sigma0_vv_wet, sigma0_hh_wet] = spm_nth_order( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r_wet, mu_r, ...
    use_F_terms, use_exp_in_I);

% --- 干燥沥青 (对比参考) ---
[sigma0_vv_dry, sigma0_hh_dry] = spm_nth_order( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r_dry, mu_r, ...
    use_F_terms, use_exp_in_I);

% --- 同时计算标准 Bragg SPM (用于交叉验证) ---
[sigma0_vv_wet_std, sigma0_hh_wet_std] = spm_bragg_standard( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r_wet);

fprintf('计算完成。\n\n');

%% ========================================================================
%  7. 转换为 dB
%     使用 10*log10(|σ⁰| + eps) 防止 log(0)
%     功率量 → dB 用 10*log10，幅值量 → dB 用 20*log10
% =========================================================================
to_dB = @(x) 10 * log10(max(x, eps));

sigma0_vv_wet_dB = to_dB(sigma0_vv_wet);
sigma0_hh_wet_dB = to_dB(sigma0_hh_wet);
sigma0_vv_dry_dB = to_dB(sigma0_vv_dry);
sigma0_hh_dry_dB = to_dB(sigma0_hh_dry);

sigma0_vv_wet_std_dB = to_dB(sigma0_vv_wet_std);
sigma0_hh_wet_std_dB = to_dB(sigma0_hh_wet_std);

ratio_wet_dB = to_dB(sigma0_vv_wet ./ max(sigma0_hh_wet, eps));
ratio_dry_dB = to_dB(sigma0_vv_dry ./ max(sigma0_hh_dry, eps));
ratio_wet_std_dB = to_dB(sigma0_vv_wet_std ./ max(sigma0_hh_wet_std, eps));

%% ========================================================================
%  8. 绘图
% ========================================================================
h_fig = figure('Color', 'w');

% --- 子图 1: σ⁰_vv 和 σ⁰_hh (复现论文 Figure 12) ---
subplot(2, 2, [1, 2]);
p1 = plot(phi_deg, sigma0_vv_wet_dB, 'b-',  'LineWidth', 2.5); hold on;
p2 = plot(phi_deg, sigma0_hh_wet_dB, 'r-',  'LineWidth', 2.5);
p3 = plot(phi_deg, sigma0_vv_dry_dB, 'b--', 'LineWidth', 1.5);
p4 = plot(phi_deg, sigma0_hh_dry_dB, 'r--', 'LineWidth', 1.5);
% 标准 Bragg SPM 对比 (点划线)
p5 = plot(phi_deg, sigma0_vv_wet_std_dB, 'b:', 'LineWidth', 1.2);
p6 = plot(phi_deg, sigma0_hh_wet_std_dB, 'r:', 'LineWidth', 1.2);

xlabel('入射角 \phi (deg)', 'FontSize', 13);
ylabel('\sigma^0 (dB)', 'FontSize', 13);
title(sprintf(['SPM 后向散射系数 — 复现 Viikari et al. (2009) Figure 12\n' ...
    '(24 GHz, 湿润 \\epsilon_r=%d, 干燥 \\epsilon_r=%.1f)'], ...
    epsilon_r_wet, epsilon_r_dry), 'FontSize', 13, 'FontWeight', 'bold');
legend([p1, p2, p3, p4, p5], {...
    '\sigma^0_{vv}  湿润 (n阶SPM)', ...
    '\sigma^0_{hh}  湿润 (n阶SPM)', ...
    '\sigma^0_{vv}  干燥 (n阶SPM)', ...
    '\sigma^0_{hh}  干燥 (n阶SPM)', ...
    '\sigma^0_{vv}  湿润 (标准Bragg对比)'}, ...
    'Location', 'southwest', 'FontSize', 9);
grid on; set(gca, 'FontSize', 12, 'GridAlpha', 0.3);
xlim([0, 80]);

% --- 子图 3: 极化比 σ⁰_vv/σ⁰_hh (湿润 vs 干燥) ---
subplot(2, 2, 3);
plot(phi_deg, ratio_wet_dB, 'b-', 'LineWidth', 2.5); hold on;
plot(phi_deg, ratio_dry_dB, 'r-', 'LineWidth', 2.5);
plot(phi_deg, ratio_wet_std_dB, 'b:', 'LineWidth', 1.2);

% 标注 70° 关键点 (论文指出预测变化为 ~17 dB)
idx_70 = find(abs(phi_deg - 70) < 0.01, 1);
if ~isempty(idx_70)
    plot(70, ratio_wet_dB(idx_70), 'ko', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'y');
    text(71, ratio_wet_dB(idx_70), ...
        sprintf('%.1f dB', ratio_wet_dB(idx_70)), ...
        'FontSize', 11, 'Color', 'k', 'FontWeight', 'bold');
end

xlabel('入射角 \phi (deg)', 'FontSize', 13);
ylabel('\sigma^0_{vv} / \sigma^0_{hh} (dB)', 'FontSize', 13);
title('极化比', 'FontSize', 14, 'FontWeight', 'bold');
legend({'湿润', '干燥', '湿润 (标准Bragg)'}, ...
    'Location', 'northwest', 'FontSize', 10);
grid on; set(gca, 'FontSize', 12, 'GridAlpha', 0.3);
xlim([0, 80]);

% --- 子图 4: 与标准 Bragg SPM 的差异 ---
subplot(2, 2, 4);
delta_vv_dB = sigma0_vv_wet_dB - sigma0_vv_wet_std_dB;
delta_hh_dB = sigma0_hh_wet_dB - sigma0_hh_wet_std_dB;
plot(phi_deg, delta_vv_dB, 'b-', 'LineWidth', 1.5); hold on;
plot(phi_deg, delta_hh_dB, 'r-', 'LineWidth', 1.5);
plot([0, 80], [0, 0], 'k--', 'LineWidth', 0.5);
xlabel('入射角 \phi (deg)', 'FontSize', 13);
ylabel('Δσ^0 (dB)', 'FontSize', 13);
title('n阶SPM − 标准Bragg SPM', 'FontSize', 14, 'FontWeight', 'bold');
legend({'\sigma^0_{vv} 差异', '\sigma^0_{hh} 差异'}, ...
    'Location', 'best', 'FontSize', 10);
grid on; set(gca, 'FontSize', 12, 'GridAlpha', 0.3);
xlim([0, 80]);

sgtitle(sprintf([ ...
    '表面参数 (Table I):  \\gamma_1=%.4f mm,  L_1=%.4f mm,  ' ...
    '\\gamma_2=%.4f mm,  L_2=%.4f mm  |  ' ...
    'F_{pp}=%s,  exp\\_in\\_I=%s'], ...
    gamma1_mm, L1_mm, gamma2_mm, L2_mm, ...
    ternary_str(use_F_terms, 'on', 'off'), ...
    ternary_str(use_exp_in_I, 'on', 'off')), ...
    'FontSize', 9, 'FontWeight', 'normal');

% 保存图像
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
if isempty(script_dir), script_dir = pwd; end
saveas(h_fig, fullfile(script_dir, 'fig_spm_full_model_results.png'));
fprintf('图像已保存至: %s\n\n', ...
    fullfile(script_dir, 'fig_spm_full_model_results.png'));

%% ========================================================================
%  9. 数值结果表格
% =========================================================================
check_angles = [0, 10, 20, 30, 40, 50, 60, 65, 70, 75, 80];

fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║  湿润沥青 (ε_r=33) — n阶SPM 后向散射系数 (dB)  ║\n');
fprintf('╠══════╤═════════════╤═════════════╤══════════════╣\n');
fprintf('║ φ(°) │  σ°_vv(dB)  │  σ°_hh(dB)  │  ratio (dB)  ║\n');
fprintf('╟──────┼─────────────┼─────────────┼──────────────╢\n');
for i = 1:length(check_angles)
    [~, idx] = min(abs(phi_deg - check_angles(i)));
    fprintf('║ %4d │ %10.2f  │ %10.2f  │  %10.2f  ║\n', ...
        check_angles(i), sigma0_vv_wet_dB(idx), ...
        sigma0_hh_wet_dB(idx), ratio_wet_dB(idx));
end
fprintf('╚══════╧═════════════╧═════════════╧══════════════╝\n\n');

fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║  干燥沥青 (ε_r=2.6) — n阶SPM 后向散射系数 (dB) ║\n');
fprintf('╠══════╤═════════════╤═════════════╤══════════════╣\n');
fprintf('║ φ(°) │  σ°_vv(dB)  │  σ°_hh(dB)  │  ratio (dB)  ║\n');
fprintf('╟──────┼─────────────┼─────────────┼──────────────╢\n');
for i = 1:length(check_angles)
    [~, idx] = min(abs(phi_deg - check_angles(i)));
    fprintf('║ %4d │ %10.2f  │ %10.2f  │  %10.2f  ║\n', ...
        check_angles(i), sigma0_vv_dry_dB(idx), ...
        sigma0_hh_dry_dB(idx), ratio_dry_dB(idx));
end
fprintf('╚══════╧═════════════╧═════════════╧══════════════╝\n\n');

%% ========================================================================
%  10. 与论文的对比验证
% =========================================================================
fprintf('═══════════ 与论文 Figure 12 的对比验证 ═══════════\n\n');

fprintf('论文关键数据点:\n');
fprintf('  a) 近垂直入射: σ⁰_vv ≈ σ⁰_hh (极化比 ≈ 0 dB)\n');
fprintf('     本模型: ratio(0°) = %.1f dB\n\n', ratio_wet_dB(1));

fprintf('  b) 湿润沥青 65° 实测范围 (论文 Fig.2):\n');
fprintf('     σ⁰_vv ≈ -20~-22 dB, σ⁰_hh ≈ -35~-38 dB\n');
fprintf('     本模型: σ⁰_vv = %.1f dB, σ⁰_hh = %.1f dB\n', ...
    sigma0_vv_wet_dB(phi_deg == 65), sigma0_hh_wet_dB(phi_deg == 65));
fprintf('     标准Bragg: σ⁰_vv = %.1f dB, σ⁰_hh = %.1f dB\n\n', ...
    sigma0_vv_wet_std_dB(phi_deg == 65), sigma0_hh_wet_std_dB(phi_deg == 65));

fprintf('  c) 大入射角极化比 (论文 IV-C 节):\n');
fprintf('     论文预测 70° 湿润沥青 σ⁰_vv/σ⁰_hh ≈ +17 dB\n');
fprintf('     (论文: "The predicted change of 17 dB at 70°\n');
fprintf('      incidence, however, was larger than the\n');
fprintf('      measured one (7 dB).")\n');
fprintf('     本模型 70° ratio = %.1f dB\n', ratio_wet_dB(idx_70));
fprintf('     标准Bragg 70° ratio = %.1f dB\n\n', ratio_wet_std_dB(idx_70));

fprintf('  d) 实测 vs 预测差异:\n');
fprintf('     论文报告: 70° 极化比预测 17 dB, 实测约 7 dB\n');
fprintf('     差异来源: SPM 假设均匀介质，实际沥青的非均匀性\n');
fprintf('              和体散射贡献使差异缩小。\n\n');

fprintf('═══════════════════════════════════════════════════\n');

%% ========================================================================
%%  核心函数 1: n 阶 SPM 后向散射系数 (论文 Eq.9-18)
%% ========================================================================
function [sigma0_vv, sigma0_hh] = spm_nth_order( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r, mu_r, ...
    use_F_terms, use_exp_in_I)
% SPM_NTH_ORDER  论文 Eq.(9)-(18) 的完整 n 阶 SPM 实现
%
% 公式摘要:
%   σ⁰_pp = (k²/2)·exp(-2k²_z·γ²)·Σ_{n=1}^{∞} |I^n_pp|²·W^(n)(-2k_x)/n!
%   其中 I^n_pp 由 Eq.(10) 给出，W^(n) 由 Eq.(17-18) 给出。
%
%   【|I|² 修正】论文 OCR 文本中 Eq.(9) 的 |I^n_pp| 按功率量纲分析
%   应为 |I^n_pp|²。标准 Fung SPM (参考[21]) 使用平方形式。
%   此处采用平方形式以保持与标准 SPM 的一致性。
%
%   【F_pp 修正项说明】Eq.(13-14) 的 F_pp 项在标准 SPM 中为高阶
%   修正。对于非磁性材料 (μ_r=1)，F_hh 的第二项含 (ε_r-1)/cos²φ，
%   在大入射角 (cosφ→0) 处发散。这是 OCR 疑似错误（原文可能包含
%   额外的归一化因子），而非物理效应。默认禁用 F_pp 项仅使用 Bragg
%   共振项 f_pp，与标准 SPM 一致。
%
%   【exp 因子说明】Eq.(10) 第一个项中包含 exp(-k²_z·γ²)。
%   在标准 Fung SPM 中，指数衰减由求和号外的 exp(-2k²_z·γ²)
%   统一处理，I^n 项内不再重复。此处提供开关以对比两种形式。

    N_phi = length(phi);

    % --- 角度相关量 ---
    cos_phi = cos(phi);
    sin_phi = sin(phi);
    sin2_phi = sin_phi.^2;
    cos2_phi = cos_phi.^2;

    k_z = k * cos_phi;    % k_z = k·cos(φ)
    k_x = k * sin_phi;    % k_x = k·sin(φ)
    K_val = -2 * k_x;     % 后向散射波数 K = -2k_x (论文 Eq.9)

    % --- 粗糙度统计量 ---
    gamma_sq = gamma1^2 + gamma2^2;   % γ² (m²)
    gamma_rms = sqrt(gamma_sq);       % γ (m)

    a_w = gamma1^2 / gamma_sq;        % 大尺度权重 a = γ²₁/γ²
    b_w = gamma2^2 / gamma_sq;        % 小尺度权重 b = γ²₂/γ²

    % --- 菲涅尔反射系数 (论文 Eq.15-16) ---
    % 注意: 论文的 R_hh 实际对应标准 Fresnel 的 R_∥ (平行/VV)
    %       R_vv 对应标准 Fresnel 的 R_⊥ (垂直/HH)
    %       这与标准记号相反！以下保持论文的记号。
    sqrt_arg = mu_r * epsilon_r - sin2_phi;
    sqrt_term = sqrt(max(sqrt_arg, 0));

    % Eq.(15): R_hh (论文记号 = 标准 Fresnel R_∥)
    R_hh = (epsilon_r * cos_phi - sqrt_term) ./ ...
           (epsilon_r * cos_phi + sqrt_term + eps);

    % Eq.(16): R_vv (论文记号 = 标准 Fresnel R_⊥)
    R_vv = (mu_r * cos_phi - sqrt_term) ./ ...
           (mu_r * cos_phi + sqrt_term + eps);

    % --- Bragg 共振系数 f_pp (论文 Eq.11-12) ---
    % f_vv =  2·R_hh/cosφ  (论文记号: R_hh=R_∥ → VV 极化)
    % f_hh = -2·R_vv/cosφ  (论文记号: R_vv=R_⊥ → HH 极化)
    f_vv =  2 * R_hh ./ cos_phi;
    f_hh = -2 * R_vv ./ cos_phi;

    % --- F_pp 修正项 (论文 Eq.13-14) ---
    % 【警告】F_hh 在 μ_r=1 时含 (ε_r-1)/cos²φ 项，大入射角发散
    if use_F_terms
        % Eq.(13)
        bracket_vv_1 = 2 * sin2_phi .* (1 + R_vv).^2 ./ cos_phi;
        bracket_vv_2 = (1 - 1/epsilon_r) ...
            + (mu_r * epsilon_r - sin2_phi - epsilon_r * cos2_phi) ...
              ./ (epsilon_r^2 * cos2_phi + eps);
        F_vv_sum = bracket_vv_1 .* bracket_vv_2;

        % Eq.(14)
        bracket_hh_1 = 2 * sin2_phi .* (1 + R_hh).^2 ./ cos_phi;
        bracket_hh_2 = (1 - 1/mu_r) ...
            + (mu_r * epsilon_r - sin2_phi - mu_r * cos2_phi) ...
              ./ (mu_r^2 * cos2_phi + eps);
        F_hh_sum = bracket_hh_1 .* bracket_hh_2;
    else
        F_vv_sum = zeros(N_phi, 1);
        F_hh_sum = zeros(N_phi, 1);
    end

    % --- 指数衰减因子 ---
    exp_factor_outside = exp(-2 * k_z.^2 * gamma_sq);  % Eq.(9) 前置因子
    if use_exp_in_I
        exp_factor_inside = exp(-k_z.^2 * gamma_sq);   % Eq.(10) 内的 exp
    else
        exp_factor_inside = ones(N_phi, 1);            % 不计入
    end

    % --- 无穷级数求和 (论文 Eq.9) ---
    tol = 1e-8;
    n_max = 20;

    sigma0_vv = zeros(N_phi, 1);
    sigma0_hh = zeros(N_phi, 1);

    converged = false;
    for n = 1:n_max
        %% 计算 W^(n)(K) (论文 Eq.17-18) ---
        % W^(n)(K) = Σ_{m=0}^{n} C_n^m · a^(n-m)·b^m · (L²_e/2) · exp(-(K·L_e)²/4)
        % 其中 L²_e = L²₁·L²₂ / ((n-m)·L²₂ + m·L²₁)
        W_n = zeros(N_phi, 1);

        for m = 0:n
            % 二项式因子: n!/((n-m)!·m!)
            comb = factorial(n) / (factorial(n-m) * factorial(m));

            % 权重因子
            wgt = a_w^(n-m) * b_w^m;

            % 等效相关长度 L_e (论文 Eq.18)
            denom_L = (n-m) * L2^2 + m * L1^2;
            if denom_L > 0
                L_eq_sq = (L1^2 * L2^2) / denom_L;
            else
                L_eq_sq = 0;
            end
            L_eq = sqrt(L_eq_sq);

            % 高斯谱 (2D 各向同性)
            % (L²_e/2)·exp(-(K·L_e)²/4)
            spectral = (L_eq_sq / 2) .* exp(-(K_val * L_eq).^2 / 4);

            W_n = W_n + comb * wgt * spectral;
        end

        %% 计算 I^n_pp (论文 Eq.10) ---
        % I^n_pp = (2k_z·γ)^n · f_pp · exp(-k²_z·γ²) + (k_z·γ)^n · [F_sum]/2
        kz_g_n     = (k_z * gamma_rms).^n;        % (k_z·γ)^n
        twokz_g_n  = (2 * k_z * gamma_rms).^n;    % (2k_z·γ)^n

        I_vv_n = twokz_g_n .* f_vv .* exp_factor_inside ...
               + kz_g_n .* F_vv_sum / 2;
        I_hh_n = twokz_g_n .* f_hh .* exp_factor_inside ...
               + kz_g_n .* F_hh_sum / 2;

        %% 累积 σ⁰ (论文 Eq.9，使用 |I|²) ---
        %   Δσ⁰ = (k²/2)·exp(-2k²_z·γ²)·|I^n|²·W^(n)/n!
        common = (k^2 / 2) * exp_factor_outside ./ factorial(n);

        delta_vv = common .* (abs(I_vv_n).^2) .* W_n;
        delta_hh = common .* (abs(I_hh_n).^2) .* W_n;

        sigma0_vv = sigma0_vv + delta_vv;
        sigma0_hh = sigma0_hh + delta_hh;

        %% 收敛检测 ---
        rel_vv = max(abs(delta_vv) ./ max(abs(sigma0_vv), eps));
        rel_hh = max(abs(delta_hh) ./ max(abs(sigma0_hh), eps));

        if max(rel_vv, rel_hh) < tol
            converged = true;
            break;
        end
    end

    if converged
        fprintf('  [收敛] SPM 级数在 n=%d 处收敛 (tol=%.0e, ε_r=%.1f)\n', ...
            n, tol, epsilon_r);
    else
        fprintf('  [警告] SPM 级数在 n_max=%d 处未完全收敛 (ε_r=%.1f)\n', ...
            n_max, epsilon_r);
    end

    %% SPM 有效性校验 (论文 Eq.19) ---
    % (k·γ)·(k·L_eq) < 1.2·√ε_r
    L_eq_val = sqrt(gamma1^2 * L1^2 + gamma2^2 * L2^2) / gamma_rms;
    valid_param = (k * gamma_rms) * (k * L_eq_val);
    valid_limit = 1.2 * sqrt(epsilon_r);

    if valid_param < valid_limit
        fprintf('  [通过] SPM 有效性: (kγ)·(kL_eq)=%.3f < 1.2√ε_r=%.3f\n', ...
            valid_param, valid_limit);
    else
        fprintf('  [边界] SPM 有效性: (kγ)·(kL_eq)=%.3f > 1.2√ε_r=%.3f\n', ...
            valid_param, valid_limit);
    end

    sigma0_vv = max(sigma0_vv, 0);
    sigma0_hh = max(sigma0_hh, 0);
end

%% ========================================================================
%%  核心函数 2: 标准一阶 Bragg SPM (交叉验证用)
%%  基于 Ulaby et al., Microwave Remote Sensing, Vol.II, 1982
%%  以及 Fung (1994) 的标准一阶 Bragg SPM
%% ========================================================================
function [sigma0_vv, sigma0_hh] = spm_bragg_standard( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r)
% SPM_BRAGG_STANDARD  标准一阶 Bragg SPM 后向散射系数
%
% 公式:
%   σ⁰_pp = 8·k⁴·γ²·cos⁴φ·|α_pp|²·W_s(K)·exp(-2k²_z·γ²)
%
% 其中 α_pp 为标准 Bragg 极化系数 (Ulaby et al. 1982, Vol.II, p.957):
%   α_hh = (ε_r - 1) / [cosφ + √(ε_r - sin²φ)]²
%   α_vv = (ε_r - 1)[sin²φ - ε_r(1 + sin²φ)] / [ε_r·cosφ + √(ε_r - sin²φ)]²
%
% 此函数用于交叉验证 n 阶 SPM 的正确性。

    cos_phi = cos(phi);
    sin_phi = sin(phi);
    sin2_phi = sin_phi.^2;

    k_z = k * cos_phi;
    K_val = -2 * k * sin_phi;  % 后向散射波数

    gamma_sq = gamma1^2 + gamma2^2;
    a_w = gamma1^2 / gamma_sq;
    b_w = gamma2^2 / gamma_sq;

    sqrt_term = sqrt(max(epsilon_r - sin2_phi, 0));

    % 标准 Bragg 极化系数
    eps_m1 = epsilon_r - 1;
    alpha_vv = eps_m1 .* (sin2_phi - epsilon_r * (1 + sin2_phi)) ...
               ./ (epsilon_r * cos_phi + sqrt_term).^2;
    alpha_hh = eps_m1 ./ (cos_phi + sqrt_term).^2;

    % 双尺度表面功率谱 W_s(K) (n=1 的 W^(1))
    W_s = a_w * (L1^2 / 2) .* exp(-(K_val * L1).^2 / 4) ...
        + b_w * (L2^2 / 2) .* exp(-(K_val * L2).^2 / 4);

    % 标准一阶 Bragg SPM
    exp_factor = exp(-2 * k_z.^2 * gamma_sq);
    prefactor = 8 * k^4 * gamma_sq * cos_phi.^4;

    sigma0_vv = prefactor .* abs(alpha_vv).^2 .* W_s .* exp_factor;
    sigma0_hh = prefactor .* abs(alpha_hh).^2 .* W_s .* exp_factor;

    sigma0_vv = max(sigma0_vv, 0);
    sigma0_hh = max(sigma0_hh, 0);
end

%% ========================================================================
%%  辅助函数: 三元运算符
%% ========================================================================
function s = ternary_str(cond, str_true, str_false)
    if cond
        s = str_true;
    else
        s = str_false;
    end
end
