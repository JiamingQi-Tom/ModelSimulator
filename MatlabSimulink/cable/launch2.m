% ������ʱ��
clc;clear;close all
% ɾ�����еĶ�ʱ�������´���һ����ʱ��
stop(timerfind);
delete(timerfind);   
timer_id = timer;
timer_id.StartDelay = 1.0;
timer_id.Period = 0.05;
% ������ִ��,fixedSpacingģʽ
timer_id.ExecutionMode = 'fixedSpacing';
timer_id.TimerFcn = @timer_handler;
%������ʱ��
start(timer_id);
tic
function timer_handler(~,~)
    persistent counter;
    if isempty(counter)
     counter = 0;
    end
    toc
%     fprintf(1,'��ʱ���ص�=%d\n',counter);
    counter = counter+1;
end