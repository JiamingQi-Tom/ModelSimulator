% 启动定时器
clc;clear;close all
% 删除现有的定时器，重新创建一个定时器
stop(timerfind);
delete(timerfind);   
timer_id = timer;
timer_id.StartDelay = 1.0;
timer_id.Period = 0.05;
% 周期性执行,fixedSpacing模式
timer_id.ExecutionMode = 'fixedSpacing';
timer_id.TimerFcn = @timer_handler;
%启动定时器
start(timer_id);
tic
function timer_handler(~,~)
    persistent counter;
    if isempty(counter)
     counter = 0;
    end
    toc
%     fprintf(1,'定时器回调=%d\n',counter);
    counter = counter+1;
end