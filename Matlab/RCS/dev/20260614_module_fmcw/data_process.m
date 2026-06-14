%% 主要程序

%% AWR2243雷达回波数据处理程序，可以完成测距、测速、测角（方位）
% 官方手册给出AWR2243雷达的方位角范围为（-60°~60°）、分辨率为1.4°，俯仰角范围为（-30°~30°）、分辨率为18°
% 因为俯仰角精度不佳，所以本代码俯仰角处理仅作参考（有但被注释掉）

clear;clc;close all

%% 参数设置
% ======================================================================================%
c = 3e8;                                                                    %光速
f0 = 77e9;                                                                  %载频 77GHz
lambda = c / f0;                                                            %波长
idle_t = 340e-6;                                                            %空闲时间
start_t = 6e-6;                                                             %发射信号起始位置
end_t = 160e-6;                                                             %发射信号结束位置
% Tc = end_t - start_t;
Tp = idle_t + end_t;
gama = 24e12;                                                               %调频斜率 Hz/s
fs = 4e6;                                                                   %采样率(Hz)
% B = mu*(end_t-start_t)*10e12;

% 回波参数配置
numSamplePerChirp = 512;                                                    %chirp内采样数
numLoops = 64;                                                              %一次发射轮询周期chirp数
numChirpPerLoop = 12;                                                       %一帧内轮询次数
numDevice = 4;                                                              %芯片个数
numRXPerDevice = 4;                                                         %每个芯片接收天线个数
numTXPerDevice = 3;                                                         %每个芯片发射天线个数
Tc = numSamplePerChirp/fs;
frameIdx = 4;                                                               %当前回波数据是第几帧

% 距离分辨率与速度分辨率
% Rres = c / (2*B);
Rres = c*fs/(2*gama*512);
Vres = lambda / (2*numLoops*Tp);
Ru = c*Tp/2;
Vu = lambda / (4*Tp);

% 距离轴与速度轴
R_axis = (0:numSamplePerChirp-1)*Rres/1e-2;                                 %单位：cm
V_axis = (-1)*((-numLoops/2:numLoops/2-1)*Vres);                            %单位：m/s

% 4个收发芯片，每个芯片3发4收
% 方位角用2、3、4号芯片发射和所有接收天线，构成一个虚拟线阵9*16=144（去掉重叠后，为86个虚拟线阵）
Ntx_az = (numDevice-1) * numTXPerDevice;                                    % 12 11 10 9 8 7 6 5 4
Nrx_az = numDevice * numRXPerDevice;                                        % 13 14 15 16 1 2 3 4 9 10 11 12 5 6 7 8
N_az_virtual = Ntx_az * Nrx_az;                                             %方位角虚拟天线个数
% 俯仰角用1号芯片3发4收
Ntx_el = (numDevice-3) * numTXPerDevice + 1;                                % 10 3 2 1
Nrx_el = (numDevice-3) * numRXPerDevice;                                    % 1 2 3 4
N_el_virtual = Ntx_el * Nrx_el;                                             %俯仰角虚拟天线个数

% 阵元间距（单位：lambda）
d_az_tx1 = 2;                                                               % 发射天线方位维阵元间距(测方位角)
d_az_tx2 = 0.5;                                                             % 发射天线方位维阵元间距(测俯仰角)
d_el_tx = [0, 0.5, 2, 3];                                                   % 发射天线俯仰维阵元间距(测俯仰角)
d_az_rx1 = 0.5;                                                             % 接收天线方位维阵元间距
d_az_rx2 = 4;                                                               % 接收天线方位维阵元间距
d_az_rx3 = 16;                                                              % 接收天线方位维阵元间距

% 天线位置（测方位角）
az_tx_pos = (0:Ntx_az-1) * d_az_tx1;                                        % 发射天线相对位置
rx_pos_perDevice = (0:numRXPerDevice-1) * d_az_rx1;                         % 单个芯片接收天线相对位置
az_rx_pos = [rx_pos_perDevice, 1.5+d_az_rx2+rx_pos_perDevice, ...    
             3+d_az_rx2+d_az_rx3+rx_pos_perDevice, ...
             5+d_az_rx2+d_az_rx3+rx_pos_perDevice];                         % 接收天线位置

% 天线位置（测俯仰角）
el_tx_pos = (0:Ntx_el-1)*d_az_tx2;                                          % 发射天线位置
el_rx_pos = (0:Nrx_el-1)*d_az_rx1;                                          % 接收天线位置

% 计算方位维虚拟阵元位置
virtual_array_az_orign = zeros(1, N_az_virtual);                            % 方位维虚拟阵元位置
for tx_idx = 1:Ntx_az
    virtual_array_az_orign((tx_idx-1)*Nrx_az+1:(tx_idx)*Nrx_az) = az_tx_pos(tx_idx) + az_rx_pos;
end

% 去掉重叠阵元后的方位维虚拟阵元位置
virtual_array_az = unique(virtual_array_az_orign) * lambda;                 % unique：去除重复元素

%% 加载数据
% ======================================================================================%
location = '.\orign_data';
data = read_orign_data(location, frameIdx, numSamplePerChirp, numLoops, numChirpPerLoop, gama, fs);
fprintf('数据加载完成！\n');
fprintf('============================================================================\n\n');

%% 目标参数
% ======================================================================================%
% 目标实测数据(有两组数据)
% 距离（m），速度（m/s），方位角（°），俯仰角（°）
targets = [2.16,    0,      0,      0; ...
           2.79,    0,      -29.1,  0];
num_targets = size(targets, 1);

fprintf('预设目标的实测数据为：\n');
for t_idx = 1:num_targets
    fprintf('目标%d的距离:%.2f(cm),速度:%.2f(m/s),方位角:%.2f(度),俯仰角:%.2f(度)\n', ...
        t_idx, targets(t_idx, 1)*1e2, targets(t_idx, 2), targets(t_idx, 3), targets(t_idx, 4));
end
fprintf('============================================================================\n\n');

%% 数据提取
% ======================================================================================%
% 原始回波存放顺序
% 512*64*16*12
% RxForMIMOProcess = [13 14 15 16 1 2 3 4 9 10 11 12 5 6 7 8];
% TxForMIMOProcess = [12 11 10 9 8 7 6 5 4 3 2 1];
% RxForMIMOProcessAz = [13 14 15 16 1 2 3 4 9 10 11 12 5 6 7 8];
% TxForMIMOProcessAz = [12 11 10 9 8 7 6 5 4];
% RxForMIMOProcessEl = [1 2 3 4];
% TxForMIMOProcessEl = [3 2 1];

% 取出来需要用到的回波数据
% data 大小为 numSamplePerChirp * numLoops * 16 * 12
% 方位维（测距、测速、方位维测角）
All_echo_az = data(:, :, :, 1:9);
Echo_az = zeros(numSamplePerChirp, numLoops, N_az_virtual);

for tx_idx = 1:Ntx_az
    Echo_az(:, :, (tx_idx-1)*Nrx_az+1:tx_idx*Nrx_az) = squeeze(All_echo_az(:, :, :, tx_idx));
end
%俯仰维（只用来做俯仰维测角）
% All_echo_el = data(:, :, 5:8, 10:12);
% Echo_el = squeeze(mean(All_echo_el, 3));

% 波形显示
figure('Name', '单个Chirp回波波形显示');
plot(1:numSamplePerChirp, mag2db(abs(squeeze(Echo_az(:, 1, 1, 1)))), 'k');
title('第一个Chirp回波');
xlabel('距离门');
ylabel('幅度 (dB)');
grid on;

%% 加窗
% 提高检测弱信号的能力
% 方位维
% win=hamming(numSamplePerChirp);
% win_mat=repmat(win,1,numLoops,N_az_virtual);
% echo_az = Echo_az .* win_mat;
echo_az = Echo_az;

%% 距离处理
% ======================================================================================%
% 距离fft
range_fft = fft(echo_az, numSamplePerChirp, 1);

% 取所有虚拟阵列和多普勒维的距离门均值为距离谱
range_map = squeeze(mean(mean(abs(range_fft), 2), 3));

figure('Name', '距离FFT结果');
plot(R_axis, mag2db(range_map), 'k');
xlabel('距离 (cm)');
ylabel('幅度 (dB)');
grid on;


%% 一维CFAR
% ======================================================================================%
% 参数设置
Pfa1 = 1e-5;                                                                %设置虚警概率
Tr1 = 15;                                                                   %距离维参考单元数（单边）
Gr1 = 10;                                                                   %距离维保护单元数（单边）

% 计算单元数量
CFAR1_N = 2 * (Tr1+Gr1) + 1;                                                %总的单元
CFAR1_N_Pro = 2 * Gr1;                                                      %保护单元
CFAR1_N_Ref = CFAR1_N - CFAR1_N_Pro - 1;                                    %参考单元

% 计算阈值因子 alpha
ca_alpha1 = CFAR1_N_Ref * (Pfa1^(-1/CFAR1_N_Ref) - 1);                      %CA-CFAR 阈值因子

% 预设阈值空间
CA_CFAR1_threshold = zeros(1, numSamplePerChirp-CFAR1_N_Ref-CFAR1_N_Pro);
CA_CFAR1_result = zeros(1, numSamplePerChirp);
range_bin = zeros(1, numSamplePerChirp);
range_target_num = 0;

% 依次扫描所有距离门
for r_idx = Tr1+Gr1+1:numSamplePerChirp-Tr1-Gr1
    % 提取当前CUT值
    CUT = abs(range_map(r_idx));

    % 构建参考单元区域
    CFAR1_ref_left = range_map(r_idx-Tr1-Gr1:r_idx-Gr1-1);
    CFAR1_ref_right = range_map(r_idx+Gr1+1:r_idx+Gr1+Tr1);
    CFAR1_ref = [CFAR1_ref_left, CFAR1_ref_right];
    
    % 计算参考单元均值
    ref_mean = sum(CFAR1_ref(:)) / CFAR1_N_Ref;

    % 计算检测阈值
    ca_threshold = ref_mean * ca_alpha1;
    CA_CFAR1_threshold(r_idx-Tr1-Gr1) = ca_threshold;

    % CA-CFAR检测结果
    if(CUT > ca_threshold)
        CA_CFAR1_result(r_idx) = 1;
        range_bin(range_target_num+1) = r_idx;
        range_target_num = range_target_num + 1;
    end
end

% 波形阈值显示
startCell = Tr1 + Gr1;
CFAR_R_axis = (startCell+1:numSamplePerChirp-startCell) * Rres / 1e-2;

hold on;
title('CA-CFAR检测结果');
plot(CFAR_R_axis, mag2db(abs(squeeze(CA_CFAR1_threshold))), 'r');
hold off;


%% MTI（可选）
mti = mti_process(range_fft, numSamplePerChirp, numLoops, N_az_virtual, Rres, Vres);

%% 速度处理
% ======================================================================================%
% 速度fft
doppler_fft = fftshift(fft(range_fft, numLoops, 2), 2);

% 取所有虚拟阵列的均值为R-D谱
doppler_map = squeeze(mean(abs(doppler_fft), 3));

% 显示所有一维CFAR检测到的目标速度
figure('Name', '目标速度位置显示');
legend_labels = cell(1, range_target_num);
hold on;
for t_idx = 1:range_target_num
    plot(V_axis, mag2db(doppler_map(range_bin(t_idx), :)));
    legend_labels{t_idx} = sprintf('距离%.2fcm', R_axis(range_bin(t_idx)));
end
hold off;
xlabel('速度位置');
ylabel('幅度 (dB)');
title('速度FFT结果');
legend(legend_labels, 'Location', 'best');
grid on;

figure('Name', '三维R-D图');
surf(R_axis, V_axis, mag2db(abs(doppler_map.')), 'EdgeColor', 'none');
title('回波的RD图');
xlabel('距离(cm)');
ylabel('速度(m/s)');
zlabel('幅度');

%% 二维CFAR
% ======================================================================================%
% 参数设置
Pfa2 = 1e-6;                                                                %虚警概率
Tr2 = 3;                                                                    %距离维参考单元数（单边）
Td2 = 1;                                                                    %多普勒维参考单元数（单边）
Gr2 = 2;                                                                    %距离维保护单元数（单边）
Gd2 = 0;                                                                    %多普勒维保护单元数（单边）

% 计算单元数量
CFAR2_N = (2*Tr2 + 2*Gr2 + 1) * (2*Td2 + 2*Gd2 + 1);                        %总的单元
CFAR2_N_Pro = (2*Gr2 + 1)*(2*Gd2 + 1);                                      %保护单元
CFAR2_N_Ref = CFAR2_N - CFAR2_N_Pro;                                        %参考单元

% 计算阈值因子
ca_alpha2 = CFAR2_N_Ref * (Pfa2^(-1/CFAR2_N_Ref) - 1);                      %CA-CFAR阈值因子

% 预设阈值空间
CA_CFAR2_threshold = zeros(numSamplePerChirp, numLoops);

% 预设结果空间
CA_CFAR2_result = zeros(numSamplePerChirp, numLoops);

% 按照Chirp数、距离门数依次扫描所有回波
for d_idx = Td2+Gd2+1:numLoops-Td2-Gd2
    for r_idx = Tr2+Gr2+1:numSamplePerChirp-Tr2-Gr2
        % 提取当前CUT值
        CUT = abs(doppler_map(r_idx, d_idx));
        
        % 构建参考单元区域
        CFAR2_ref = abs(doppler_map(r_idx-Tr2-Gr2:r_idx+Tr2+Gr2, d_idx-Td2-Gd2:d_idx+Td2+Gd2));
        
        % 去除保护单元
        CFAR2_ref(Tr2+1:Tr2+Gr2+Gr2+1, Td2+1:Td2+Gd2+Gd2+1) = 0;
        
        % 计算参考单元均值（去除零值）
        ref_mean = sum(CFAR2_ref(:)) / CFAR2_N_Ref;
        
        % 计算检测阈值
        ca_threshold = ref_mean * ca_alpha2;
        CA_CFAR2_threshold(r_idx-Tr2-Gr2, d_idx-Td2-Gd2) = ca_threshold;
        
        % 目标检测
        if CUT > ca_threshold
            CA_CFAR2_result(r_idx, d_idx) = 1;
        end
    end
end

% 从MTD结果中找到峰值，峰值的多普勒和距离维度即为目标所在多普勒和距离点
pks = imregionalmax(abs(doppler_map));
target = pks & CA_CFAR2_result;
[target_R, target_V] = find(target);

% 判断峰值检测是否正确
num_target_V = length(target_V);
num_target_R = length(target_R);

if num_target_V ~= num_target_R
    fprintf('目标峰值检测 Error！！！\n');
    pause;
end

num_target = num_target_V;

% 检测结果标记
figure('Name', '二维CFAR结果'); 
imagesc(R_axis, V_axis, abs(target.'));
title('二维CFAR检测结果');
xlabel('距离(cm)');
ylabel('速度(m/s)');
zlabel('幅度');
colorbar;
% 标记目标
hold on;
plot(R_axis(target_R), V_axis(target_V), 'ro');
hold off;

%% 相位补偿 (多普勒补偿)
% estimated_doppler = -V_axis(target_V);
                    
% for i=1:num_target
% 
%     %提取距离门
%     doppler_matrix=squeeze(range_fft(max_r(i),:,:));
% 
%     % 多普勒fft(+fftshift)
%     doppler_fft = fftshift(fft(doppler_matrix, [], 1), 1);                  %fft(X,num,dim) fftshift(X,dim)
% 
%     % 初始化矩阵
%     compensated_doppler = zeros(size(doppler_fft));
% 
%     for m=1:Nchirp
%         for n=1:num_vx
% 
%             %相位补偿
%             phase_compensated=exp(-1j*2*pi *(n-1)* 2*estimated_doppler(i)*Tc/lamda);
%             compensated_doppler(m,n) = doppler_fft(m,n) * phase_compensated;
%         end
%     end
% end
   

%% 角度估计 (DBF)
% ======================================================================================%
az_angle_range = -60:0.1:60;                                                %方位角距离范围

% 初始化矩阵
az_angle_spectrum = zeros(size(az_angle_range));

% 根据虚拟阵列位置排序数据
virtual_array_az_orign_temp = zeros(2, length(virtual_array_az_orign));
virtual_array_az_orign_temp(1, :) = virtual_array_az_orign;
for array_idx = 1:length(virtual_array_az_orign)
    virtual_array_az_orign_temp(2, array_idx) = array_idx;
end
virtual_array_az_orign_temp = sortrows(virtual_array_az_orign_temp', 1)';
az_angle_data_temp = zeros(size(doppler_fft));
for array_idx = 1:length(virtual_array_az_orign)
    az_angle_data_temp(:, :, array_idx) = doppler_fft(:, :, virtual_array_az_orign_temp(2, array_idx));
end

% 去除虚拟阵列重叠的单元数据
az_angle_data_temp2 = zeros(numSamplePerChirp, numLoops, length(virtual_array_az_orign));
az_angle_data = zeros(numSamplePerChirp, numLoops, length(virtual_array_az));
% 先对虚拟阵列中重叠单元数据做加权叠加（求平均）
for array_idx1 = 1:length(virtual_array_az_orign)
    if virtual_array_az_orign_temp(2, array_idx1) == 0
        continue;
    end
    az_angle_data_temp2(:, :, array_idx1) = az_angle_data_temp(:, :, array_idx1);
    factor = 1;

    for array_idx2 = array_idx1+1:length(virtual_array_az_orign)
        if virtual_array_az_orign_temp(1, array_idx1) == virtual_array_az_orign_temp(1, array_idx2)
            az_angle_data_temp2(:, :, array_idx1) = (az_angle_data_temp2(:, :, array_idx1).*factor + az_angle_data_temp2(:, :, array_idx2)) ./ (factor+1);
            factor = factor+1;
            virtual_array_az_orign_temp(2, array_idx2) = 0;
        end

        if virtual_array_az_orign_temp(1, array_idx1) < virtual_array_az_orign_temp(1, array_idx2)
            break;
        end
    end 
end
% 去掉没用的虚拟阵列
idx = 1;
for array_idx = 1:length(virtual_array_az_orign)
    if virtual_array_az_orign_temp(2, array_idx) == 0
        continue;
    end
    az_angle_data(:, :, idx) = az_angle_data_temp2(:, :, array_idx);
    idx = idx + 1;
end

figure('Name' ,'DBF角度谱');
hold on;
for t_idx = 1:num_target
    % 提取目标距离-多普勒单元数据
    % 方位维
    az_angle_data_current = squeeze(az_angle_data(target_R(t_idx), target_V(t_idx), :));

    % DBF波束形成
    for az_angle_idx = 1:length(az_angle_range)
        % 转换当前波束角为弧度
        az_angle = deg2rad(az_angle_range(az_angle_idx));

        % 合成当前波束角度的平面阵列导向矢量
        az_vector = exp(1j*2*pi * virtual_array_az * sin(az_angle) / lambda);

        % 计算当前波束下目标的功率幅度
        az_angle_spectrum(az_angle_idx) = abs(az_vector * az_angle_data_current)^2;
    end

    % 归一化角度谱
    az_angle_spectrum_db = mag2db(az_angle_spectrum/max(az_angle_spectrum));

    % 绘制角度谱
    [az_M,az_I]=max(az_angle_spectrum_db);
    plot(az_angle_range, az_angle_spectrum_db);
    xlabel('角度 (度)');
    ylabel('归一化幅度 (dB)');
    title('DBF角度谱(方位角)');
    grid on;

    % 打印目标信息
    fprintf('检测到的目标%d的距离：%.2f(cm),速度：%.2f(m/s),方位角：%.2f(度)\n', t_idx, ...
        R_axis(target_R(t_idx)), V_axis(target_V(t_idx)), az_angle_range(az_I));
end




