clear; clc; close all
%% 常数
g = 9.81;  R = 10;  dt = 0.1;  tEnd = 8;
t = 1.1:dt:tEnd;
x1_0 = 17800;  y1_0 = 0;  z1_0 = 1800;
XM_0 = 20000;  YM_0 = 0;  ZM_0 = 2000;
VMx = 300 * XM_0/norm([XM_0,YM_0,ZM_0]);
VMy = 300 * YM_0/norm([XM_0,YM_0,ZM_0]);
VMz = 300 * ZM_0/norm([XM_0,YM_0,ZM_0]);
rCyl = 7;  hCyl = 10;  nSample = 50;
Vsmoke = 3;

%% 退火参数
T0 = 500;  Tf = 1e-3;  alpha = 0.9;  maxIter = 10;
% lb = [ 70, 0, 0, 0];   % [v1, alpha1, T1, dT]
% ub = [140,  2*pi, 5.0, 8.0];
lb = [ 70, 0, 0, 0];   % [v1, alpha1, T1, dT]
ub = [140,  pi, 4.0, 4.0];
x  = (lb + ub)/2;     % 初始解

bestX = x;  bestObj = 0;
maxIterTotal = ceil(log(Tf/T0)/log(alpha))*maxIter;   % 总迭代次数
histBest   = nan(maxIterTotal,1);                   % 历史最优
idx        = 0;
%% 主循环
T = T0;
while T > Tf
    for iter = 1:maxIter
         idx = idx +1;
        newX = x + (rand(1,4)-0.5) .* (ub-lb) * 0.2;   % 小扰动
        newX = max(min(newX,ub),lb);                   % 越界修复
        obj = objFun(newX);                            % 计算遮挡时长
        delta = obj - bestObj;
        if delta > 0 || rand < exp(delta/T)
            x = newX;  bestX = newX;  bestObj = obj;
        end
        disp(bestObj);
    histBest(idx) = bestObj;
    end
    T = T*alpha;
end

histBest = histBest(1:idx);           % 去掉尾部 NaN
figure;
plot(1:idx, histBest, 'LineWidth', 2);
xlabel('退火迭代次数');
ylabel('历史最优遮挡时长 / s');
title('模拟退火收敛曲线');
grid on;

fprintf('最大遮挡时长 = %.2f s\n', bestObj);
fprintf('最优参数  v1=%.2f  α1=%.3f rad  T1=%.2f s  Δt=%.2f s\n', bestX);

%% 目标函数
function obj = objFun(par)
g = 9.81;  R = 10;  dt = 0.01;  tEnd = 30;
t = 0:dt:tEnd;
x1_0 = 17800;  y1_0 = 0;  z1_0 = 1800;
XM_0 = 20000;  YM_0 = 0;  ZM_0 = 2000;
VMx = 300 * XM_0/norm([XM_0,YM_0,ZM_0]);
VMy = 300 * YM_0/norm([XM_0,YM_0,ZM_0]);
VMz = 300 * ZM_0/norm([XM_0,YM_0,ZM_0]);
rCyl = 7;  hCyl = 10;  nSample = 200;
Vsmoke = 3;
v1 = par(1);  alpha1 = par(2);  T1 = par(3);  dT = par(4);
T2 = T1 + dT;

% 无人机轨迹
v1x = v1 * cos(alpha1);  v1y = v1 * sin(alpha1);
% x1 = x1_0 + v1x*t;     
% y1 = y1_0 + v1y*t;

% 烟幕球轨迹（爆炸后 x,y 冻结）
xsFreeze = x1_0 + v1x*T2;
ysFreeze = y1_0 + v1y*T2;

% 2) 再拼出完整轨迹
xs = (x1_0 + v1x*t) .* (t>=T1 & t<T2) + xsFreeze .* (t>=T2);
ys = (y1_0 + v1y*t) .* (t>=T1 & t<T2) + ysFreeze .* (t>=T2);
% 高度继续按原逻辑
zs = z1_0 ...
     - 0.5*g*(t-T1).^2 .* (t>=T1 & t<T2) ...
     - (0.5*g*(T2-T1)^2 + Vsmoke*(t-T2)) .* (t>=T2);

% 导弹轨迹
XM = XM_0 - VMx*t;  YM = YM_0 - VMy*t;  ZM = ZM_0 - VMz*t;

% 圆柱表面随机点（只需生成一次，缓存加速）
persistent xa ya za
if isempty(xa)
    alpha = rand(1,nSample)*2*pi;
    h     = rand(1,nSample)*hCyl;
    r     = rand(1,nSample)*rCyl;
    xa    = r .* cos(alpha);
    ya    = r .* sin(alpha) + 200;
    za    = h;
end

% 遮挡判断 + 提前退出
mark = false(size(t));
for k = 1:length(t)
    M = [XM(k), YM(k), ZM(k)];   % 导弹当前位置
    S = [xs(k), ys(k), zs(k)]; 
    % 提前退出：导弹已飞过烟幕球（横坐标判据 + 10 m 余量）
    if  XM(k) < xs(k) - 10
        % t   = t(1:k);
        % mark= mark(1:k);
        break;
    end

     % 1×nSample 同形          % 烟幕球心（x,y 冻结，z 随时间）
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
validIdx = (t >= T2) & (t <= T2+20);
obj  = sum(mark & validIdx) * dt;
end
