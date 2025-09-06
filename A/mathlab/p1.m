clear; clc; close all
%% 常数
g   = 9.81;              % m/s^2
R   = 10;                % 烟幕球半径 m
dt  = 0.1;               % 时间步长 s
% T1  = 1.5;               % 投放时刻 s
% T2  = 5.1;               % 爆炸时刻 s

T1  = 2;               % 投放时刻 s
dT = 0;
T2  = T1 + dT;
Vsmoke = 3;              % 烟幕球下沉速度 m/s
tEnd= T2 + 20;                % 模拟总时长 s
t   = 0:dt:tEnd;

%% 初始位置 / 速度
% 无人机 FY1
x1_0 = 17800;  y1_0 = 0;   z1_0 = 1800;
 u1   = 100;                % 地面速率 m/s
% u1xy = u1 * [x1_0;y1_0]/norm([x1_0;y1_0]); % 单位方向
% u1x  = u1xy(1);
% u1y  = u1xy(2);
alpha1 = 0.5;
u1x = -u1 * cos(alpha1);  u1y = -u1 * sin(alpha1);

% 导弹 M1
XM_0 = 20000;  YM_0 = 0;   ZM_0 = 2000;
VM   = 300;                % 速率 m/s
VMxyz= VM * [XM_0;YM_0;ZM_0]/norm([XM_0;YM_0;ZM_0]);
VMx  = VMxyz(1);
VMy  = VMxyz(2);
VMz  = VMxyz(3);

% 真目标圆柱参数
rCyl = 7;   hCyl = 10;   nSample = 50;  % 表面取样点数

%% 轨迹
% 无人机
x1 = x1_0 - u1x*t;
y1 = y1_0 - u1y*t;
z1 = z1_0 + 0*t;           % 等高巡航

% 导弹
XM = XM_0 - VMx*t;
YM = YM_0 - VMy*t;
ZM = ZM_0 - VMz*t;

% 烟幕球（爆炸前抛物线，爆炸后匀速下沉）
% 分段赋值：只在 T1≤t<T2 用飞行段；T≥T2 用冻结值
% 1) 先算 T2 时刻的冻结坐标
xsFreeze = x1_0 - u1x*T2;
ysFreeze = y1_0 - u1y*T2;

% 2) 再拼出完整轨迹
xs = (x1_0 - u1x*t) .* (t>=T1 & t<T2) + xsFreeze .* (t>=T2);
ys = (y1_0 - u1y*t) .* (t>=T1 & t<T2) + ysFreeze .* (t>=T2);
% 高度继续按原逻辑
zs = z1_0 ...
     - 0.5*g*(t-T1).^2 .* (t>=T1 & t<T2) ...
     - (0.5*g*(T2-T1)^2 + Vsmoke*(t-T2)) .* (t>=T2);

%% 在圆柱表面随机取点
alpha = rand(1,nSample)*2*pi;  % 随机生成 nSample 个圆周角，范围 [0, 2π]
h     = rand(1,nSample)*hCyl; % 随机生成 nSample 个高度，范围 [0, hCyl]
r     = rand(1,nSample)*rCyl; % 随机生成 nSample 个半径，范围 [0, rCyl]

% 将极坐标转换为直角坐标
xa    = r .* cos(alpha);       % x 坐标
ya    = r .* sin(alpha) + 200;       % y 坐标
za    = h;                     % z 坐标

%% 遮挡判断
mark = false(size(t));
for k = 1:length(t)
    M = [XM(k), YM(k), ZM(k)];   % 导弹当前位置
    S = [xs(k), ys(k), zs(k)];   % 烟幕球心

    % === 提前退出：导弹已飞过烟幕球 ===
    if XM(k) < xs(k) - 10 %&& ZM(k) < zs(k) +10
    % t   = t(1:k);
    % mark= mark(1:k);
    % xs  = xs(1:k);  ys = ys(1:k);  zs = zs(1:k);
    % XM  = XM(1:k);  YM = XM(1:k);  ZM = ZM(1:k);
    break;
    end
    % === 原遮挡判断 ===
    count = 0;
    for p = 1:nSample
        A = [xa(p), ya(p), za(p)];
        d = M - A;                          % 视线方向
        P = S - A;                          % A→球心
        D = norm(cross(P,d)) / norm(d);     % 点到直线距离
        if D < R
            count = count + 1;
        end
    end
    mark(k) = count / nSample >= 1;         % 100% 采样点被挡
end
%% 有效遮挡时长（仅统计爆炸后 20 s 内）
validIdx = (t >= T2) & (t <= T2+20);
effTime  = sum(mark & validIdx) * dt;

fprintf('有效遮挡时长 = %.2f s\n', effTime);

figure;
yyaxis left
plot(t, mark, 'LineWidth', 2); ylabel('遮挡逻辑');
yyaxis right
plot(t, zs, 'r'); ylabel('烟幕球高度 z_s (m)');
xlabel('时间 t (s)');
title('遮挡判断与烟幕球高度');
grid on

 xr = x1_0 - u1x*T1
yr = y1_0 - u1y*T1
zr = z1_0
xsFreeze = x1_0 - u1x*T2
ysFreeze = y1_0 - u1y*T2
zsf =z1_0 - 0.5*g*(T2-T1).^2


