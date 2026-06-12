%% AWR2243参数使用说明

clc

% 前置参数
c = 3e8; 
f0 = 77e9;
lambda = c / f0;

%% 波形参数

%% 1.设置Rres

fprintf('初始典型值Rres:0.0488 fs:4e6 miu:24e12 numSamplePerChirp:512\n');


%% 1.1修改参数
keyboard;
fs = 4e6; %fs复采样最大22.5e6, 实采样最大45e6
miu = 24e12; %miu最大值为500e12
numSamplePerChirp = 512; %numSamplePerChirp复采样时最大512,实采样时最大1024


Rres = c*fs/(2*miu*numSamplePerChirp);

fprintf('实际Rres: %.4f\n',Rres);

%% 2.设置Vres


%% 2.1修改参数
Tr = numSamplePerChirp/fs; %实际采样时间Tr
fprintf('Tr: %e, Tr应小于end_t-start_t\n',Tr); %Tr应小于end_t - start_t

keyboard;
end_t = 160e-6; %adc结束采样时间
start_t = 6e-6; %adc开始采样时间,因为频率上升稳定需要时间,一般默认即可


Tc = end_t - start_t; %理论采样时间
if Tc <= Tr
    error('Tc应略大于Tr');
end
Be = numSamplePerChirp/fs * miu;
B = miu * Tc;
if B >= 4e9
    error('B的最大带宽为4GHz');
end

%% 2.2修改参数
fprintf('B的带宽%e, 根据带宽确定idle_t(应大于minTime)\n',B);

keyboard;
idle_t = 340e-6; %空闲时间,频率下降时间Ramp Down Time
Tp = end_t+idle_t;
numChirpPerLoop = 64;

minTime(B < 1e9) = 2e-6;
minTime(B >= 1e9 && B < 2e9) = 3.5e-6;
minTime(B >= 2e9 && B < 3e9) = 5e-6;
minTime(B >= 3e9) = 7e-6;

if minTime >= idle_t
    error('idle_t应该大于minTime')
end

fprintf('初始典型值Vres:0.0609 Tp:5e-4 numChirpPerLoop:64\n');


Vres = lambda / (2*numChirpPerLoop*Tp);

fprintf('实际Vres: %.4f\n',Vres);

%% 3.实际参数

Ru = numSamplePerChirp * Rres;
Vu = lambda / (4*Tp);

fprintf('实际参数: ');
fprintf('Rres: %.4fm Vres: %.4fm/s Ru: %.2fm Vu: %.2fm/s\n',...
    Rres,Vres,Ru,Vu);

%% 4.采集参数

numTx = 12; %默认为12发

%% 4.1修改参数

fprintf('帧时间应该大于%f\n',Tp*numTx*(numChirpPerLoop+1));
keyboard;

Inter_Frame_Interval = 0.4; %帧时间应该大于Tp*numTx*(numChirpPerLoop+1)
nframes = 10; %帧数

if Inter_Frame_Interval <= Tp*numTx*(numChirpPerLoop+1)
    error('帧时间不满足条件')
end

%% 4.2修改参数 


fprintf('捕获时间应大于%.2f\n',Inter_Frame_Interval*nframes);

keyboard;
capture_time = 10; %捕获时间应该大于帧数*帧时间

if capture_time<=Inter_Frame_Interval*nframes
    error('采集时间不满足条件')
end

%% 计算数据大小

% 快时间采样点数*接收通道数*Chirp数量*每帧Loop数*帧数*I/Q复数...
%                                      *ADC位数(16位/12位(bit)=2字节(Byte))
size_data_bin = numSamplePerChirp*4*numChirpPerLoop*12*nframes*2*2/1024/1024;

fprintf('数据大小约为%.2fMB\n',size_data_bin);




