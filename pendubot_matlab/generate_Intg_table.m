%%
clc; clear all;  close all;

% First table generation 
n_dscrt = 1000;

phi0 = -pi/2; 
thta0 = 0.0;
k  = 0.5;
theta =  1.2;
theta_d = 0;

run('tests_PoC.m');

s_list = -pi:(2*pi/n_dscrt):pi;
s_d_list = -30:(60/n_dscrt):30;
I_table = zeros(length(s_list), length(s_d_list));

WaitMessage = parfor_wait(length(s_list), 'Waitbar', true);
parfor i = 1:length(s_list)
   temp = zeros(1, length(s_d_list));
   for j = 1:length(s_d_list)
    temp(j) =  Intg(s_list(i),s_d_list(j), s_str(1),s_d_str(1));
   end
   I_table(i,:) = temp;
   WaitMessage.Send;
end
WaitMessage.Destroy

path = pwd;
save('mat_files/Integ1.mat','I_table')
save([path(1:end-15),'pndbt/src/mat_files/Integ1.mat','I_table')

% Second table generation 
clc; clear all;  close all;

n_dscrt = 1000;

phi0 = -pi/2; 
thta0 = 0.0;
k  = 0.0;
theta =  0;
theta_d = 15;

run('tests_PoC.m');

s_list = -pi:(2*pi/n_dscrt):pi;
s_d_list = -30:(60/n_dscrt):30;
I_table = zeros(length(s_list), length(s_d_list));

WaitMessage = parfor_wait(length(s_list), 'Waitbar', true);
parfor i = 1:length(s_list)
   temp = zeros(1, length(s_d_list));
   for j = 1:length(s_d_list)
    temp(j) =  Intg(s_list(i),s_d_list(j), s_str(1),s_d_str(1));
   end
   I_table(i,:) = temp;
   WaitMessage.Send;
end
WaitMessage.Destroy

path = pwd;
save('mat_files/Integ2.mat','I_table')
save([path(1:end-15),'pndbt/src/mat_files/Integ2.mat','I_table')

%%
clc; clear;

load('mat_files/Integ1.mat');

[X,Y] = meshgrid(-30:0.12:30, -pi:0.0126:pi);
C = X.*Y;

h = surf(X,Y,I_table,C)
set(h,'LineStyle','none')

colorbar
