function dh_params = extractDH(robot)
    numJoints = length(robot.Bodies);
    dh_params = zeros(numJoints, 4); % 预分配 DH 参数矩阵
    for i = 1:numJoints
        body = robot.Bodies{i};
        T = body.Joint.JointToParentTransform;  % 获取变换矩阵
        [alpha, a, d, theta] = tr2dh(T);  % 计算 DH 参数
        dh_params(i, :) = [d, theta, a,  alpha];
    end
end

function [alpha, a, d, theta] = tr2dh(T)
    % 提取DH参数
    alpha = atan2(T(2,3), T(3,3));  % Z 轴扭转角
    a = T(1,4);                     % X 轴长度
    d = T(3,4);                     % Z 轴偏移
    theta = atan2(T(2,1), T(1,1));   % 旋转角度
end
