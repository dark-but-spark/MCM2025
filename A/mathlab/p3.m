clear; clc; close all
%% 退火参数
T0 = 1000;  Tf = 1e-3;  alpha = 0.9;  maxIter = 200;
lb = [ 70, 0, 0.0, 0 1 0 1 0];
ub = [140,  pi, 4.0, 4.0  4.0 4.0 4.0 4.0];
x  = (lb + ub)/2;            % 初始解
bestX = x;  bestObj = 0;

maxIterTotal = ceil(log(Tf/T0)/log(alpha))*maxIter;   % 总迭代次数
histBest   = nan(maxIterTotal,1);                   % 历史最优
idx        = 0;
%% 主循环
T = T0;
while T > Tf
    for iter = 1:maxIter
        idx = idx +1;
        newX = x + (rand(1,8)-0.5) .* (ub-lb) * 0.2;   % 小扰动
        newX = max(min(newX,ub),lb);                   % 越界修复
        obj = objFun(newX);                            % 计算遮挡时长
        delta = obj - bestObj;
        if delta > 0 || rand < exp(delta/T)
            x = newX;  bestX = newX;  bestObj = obj;
        end
        disp(bestObj);
        histBest(idx) = bestObj;      % 记录"当前历史最优"
    end
    T = T*alpha;
end
histBest = histBest(1:idx);           % 去掉尾部 NaN
figure;
plot(1:idx, histBest, 'LineWidth', 1.2);
xlabel('退火迭代次数');
ylabel('历史最优遮挡时长 / s');
title('模拟退火收敛曲线');
grid on;

fprintf('最大遮挡时长 = %.2f s\n', bestObj);
fprintf('最优参数\n v1=%.2f  α1=%.3f  T1=%.2f  dT1=%.2f  DT2=%.2f  dT2=%.2f  DT3=%.2f  dT3=%.2f\n', bestX);
%% 目标函数


function obj = objFun(par)
g = 9.81;  R0 = 10;  dt = 0.1;  tEnd = 15;
t = 0:dt:tEnd;
x1_0 = 17800;  y1_0 = 0;  z1_0 = 1800;
XM_0 = 20000;  YM_0 = 0;  ZM_0 = 2000;
VMx = 300 * XM_0/norm([XM_0,YM_0,ZM_0]);
VMy = 300 * YM_0/norm([XM_0,YM_0,ZM_0]);
VMz = 300 * ZM_0/norm([XM_0,YM_0,ZM_0]);
rCyl = 7;  hCyl = 10;  nSample = 50;
Vsmoke = 3;
v1 = par(1);  alpha1 = par(2);  
T1 = par(3);  dT1 = par(4);
DT2 = par(5);  dT2 = par(6);
DT3 = par(7);  dT3 = par(8);
T2 = T1 +DT2;
T3 = T2 +DT3;
T_1 = T1 + dT1;
T_2 = T2 +dT2;
T_3 = T3 +dT3;

% 无人机轨迹
v1x = v1 * cos(alpha1);  v1y = v1 * sin(alpha1);
% x1 = x1_0 + v1x*t;     
% y1 = y1_0 + v1y*t;

% 烟幕球1轨迹（爆炸后 x,y 冻结）
xsFreeze1 = x1_0 + v1x*T_1;
ysFreeze1 = y1_0 + v1y*T_1;

xs1 = (x1_0 + v1x*t) .* (t>=T1 & t<T_1) + xsFreeze1 .* (t>=T_1);
ys1 = (y1_0 + v1y*t) .* (t>=T1 & t<T_1) + ysFreeze1 .* (t>=T_1);
zs1 = z1_0 ...
     - 0.5*g*(t-T1).^2 .* (t>=T1 & t<T_1) ...
     - (0.5*g*(T_1-T1)^2 + Vsmoke*(t-T_1)) .* (t>=T_1);
R1 = 0.*(t < T_1) + R0.*(t>=T_1 & t <= T_1 +20) + 0.*(t > T_1 +20);
% 烟幕球2轨迹（爆炸后 x,y 冻结）
xsFreeze2 = x1_0 + v1x*T_2;
ysFreeze2 = y1_0 + v1y*T_2;

xs2 = (x1_0 + v1x*t) .* (t>=T2 & t<T_2) + xsFreeze2 .* (t>=T_2);
ys2 = (y1_0 + v1y*t) .* (t>=T2 & t<T_2) + ysFreeze2 .* (t>=T_2);
zs2 = z1_0 ...
     - 0.5*g*(t-T2).^2 .* (t>=T2 & t<T_2) ...
     - (0.5*g*(T_2-T2)^2 + Vsmoke*(t-T_2)) .* (t>=T_2);
R2 = 0.*(t < T_2) + R0.*(t>=T_2 & t <= T_2 +20) + 0.*(t > T_2 +20);

% 烟幕球3轨迹（爆炸后 x,y 冻结）
xsFreeze3 = x1_0 + v1x*T_3;
ysFreeze3 = y1_0 + v1y*T_3;

xs3 = (x1_0 + v1x*t) .* (t>=T3 & t<T_3) + xsFreeze3 .* (t>=T_3);
ys3 = (y1_0 + v1y*t) .* (t>=T3 & t<T_3) + ysFreeze3 .* (t>=T_3);
zs3 = z1_0 ...
     - 0.5*g*(t-T3).^2 .* (t>=T3 & t<T_3) ...
     - (0.5*g*(T_3-T3)^2 + Vsmoke*(t-T_3)) .* (t>=T_3);
R3 = 0.*(t < T_3) + R0.*(t>=T_3 & t <= T_3 +20) + 0.*(t > T_3 +20);
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
    S1 = [xs1(k), ys1(k), zs1(k)]; 
    S2 = [xs2(k), ys2(k), zs2(k)]; 
    S3 = [xs3(k), ys3(k), zs3(k)]; 
    % 提前退出：导弹已飞过烟幕球（横坐标判据 + 10 m 余量）

     % 1×nSample 同形          % 烟幕球心（x,y 冻结，z 随时间）
      % if XM(k) < min([xs1(k),xs2(k),xs3(k)]) - 10
      %   break;
      % end
    if XM(k) < xs1(k) -10
        R1(k) = 0;
    end
    if XM(k) < xs2(k) -10
        R2(k) = 0;
    end
    if XM(k) < xs3(k) -10
        R3(k) = 0;
    end
   cnt1 = 0;
    if R1(k)>0
        for p = 1:nSample
            A = [xa(p), ya(p), za(p)];
            d = M - A;  P = S1 - A;
            if norm(cross(P,d))/norm(d) < R1(k); cnt1 = cnt1+1; end
        end
    end
    % 球2
    cnt2 = 0;
    if R2(k)>0
        for p = 1:nSample
            A = [xa(p), ya(p), za(p)];
            d = M - A;  P = S2 - A;
            if norm(cross(P,d))/norm(d) < R2(k); cnt2 = cnt2+1; end
        end
    end
    % 球3
    cnt3 = 0;
    if R3(k)>0
        for p = 1:nSample
            A = [xa(p), ya(p), za(p)];
            d = M - A;  P = S3 - A;
            if norm(cross(P,d))/norm(d) < R3(k); cnt3 = cnt3+1; end
        end
    end

    if (cnt1/nSample >=1) || (cnt2/nSample>=1) || (cnt3/nSample>=1)
        mark(k) = true;
    end
end
obj  = sum(mark) * dt;
end
