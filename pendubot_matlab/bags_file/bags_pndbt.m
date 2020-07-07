clc; clear all; close all;
% name = 'bags/torque_A_0.3_v_1.bag';
name = 'bags/harmonic_A_0.785398163397_v_0.125.bag';
bag = rosbag(name);
end_time = 80;

bag.AvailableTopics
% extract the desired topics
start = bag.StartTime;
bagselect_current = select(bag, 'Time', [start start + end_time], 'Topic', '/current_synch' );
bagselect_torque = select(bag, 'Time', [start start + end_time], 'Topic','/netft_data_synch' );
% bagselect_command = select(bag, 'Time', [start start + end_time], 'Topic', '/command_synch' );
% bagselect_position = select(bag, 'Time', [start start + end_time], 'Topic', '/position_synch' );
% bagselect_velocity = select(bag, 'Time', [start start + end_time], 'Topic','/velocity_synch' );
% bagselect_imu = select(bag, 'Time', [start start + end_time], 'Topic','/imu_sensor_synch' );
bagselect_Jstates = select(bag, 'Time', [start start + end_time], 'Topic','/joint_states_synch' );

% msgs = readMessages(bagselect, [1 2]);
% extract message data as time series
ts_cur = timeseries(bagselect_current, 'Data');
ts_trq = timeseries(bagselect_torque,  'Wrench.Torque.Z');
% ts_cmd = timeseries(bagselect_command, 'Data');
% ts_pos = timeseries(bagselect_position, 'Data');
% ts_vel = timeseries(bagselect_velocity,  'Data');

% extract the joint states as time series
msgs = readMessages(bagselect_Jstates);
js_pos = [];
for i=  1:numel(msgs)
    js_pos = [js_pos msgs{i}.Position];
end

js_vel = [];
for i=  1:numel(msgs)
    js_vel = [js_vel msgs{i}.Velocity];
end

ts_Js_time = timeseries(bagselect_Jstates,  'Header.Stamp.Sec');
time = ts_Js_time.Time;
%
ts_Js_pos_shldr = timeseries(js_pos(1,:)',time);
ts_Js_pos_elbw = timeseries(js_pos(2,:)',time);

ts_Js_vel_shldr = timeseries(js_vel(1,:)',time);
ts_Js_vel_elbw = timeseries(js_vel(2,:)',time);

% extract the imu data as time series
% msgs = readMessages(bagselect_imu,'DataFormat','struct');
% accX_imu = [];
% accY_imu = [];
% accZ_imu = [];
% gyroX_imu = [];
% gyroY_imu = [];
% gyroZ_imu = [];
% Q0_imu = [];
% Q1_imu = [];
% Q2_imu = [];
% Q3_imu = [];
% 
% time = [];
% for i=  1:numel(msgs)
%     accX_imu = [accX_imu msgs{i}.AccelX];
%     accY_imu = [accY_imu msgs{i}.AccelY];
%     accZ_imu = [accZ_imu msgs{i}.AccelZ];
%     gyroX_imu = [gyroX_imu msgs{i}.GyroX];
%     gyroY_imu = [gyroY_imu msgs{i}.GyroY];
%     gyroZ_imu = [gyroZ_imu msgs{i}.GyroZ];
%     Q0_imu = [Q0_imu msgs{i}.Q0];
%     Q1_imu = [Q1_imu msgs{i}.Q1];
%     Q2_imu = [Q2_imu msgs{i}.Q2];
%     Q3_imu = [Q3_imu msgs{i}.Q3];
%     time = [time msgs{i}.Time];
% end
% 
% ts_accX_imu = timeseries(accX_imu,time);
% ts_accY_imu = timeseries(accY_imu,time);
% ts_accZ_imu = timeseries(accZ_imu,time);
% 
% ts_gyroX_imu = timeseries(gyroX_imu,time);
% ts_gyroY_imu = timeseries(gyroY_imu,time);
% ts_gyroZ_imu = timeseries(gyroZ_imu,time);
% 
% ts_Q0_imu = timeseries(Q0_imu,time);
% ts_Q1_imu = timeseries(Q1_imu,time);
% ts_Q2_imu = timeseries(Q2_imu,time);
% ts_Q3_imu = timeseries(Q3_imu,time);
% 

% len = min([ts_Js_vel_elbw.Length, ts_Js_vel_shldr.Length, ts_Js_pos_elbw.Length, ...
%             ts_Js_pos_shldr.Length,  ts_cur.Length, ts_trq.Length, ts_cmd.Length, ts_pos.Length, ts_vel.Length, ts_accX_imu.Length]);

% len = min([ts_Js_vel_elbw.Length, ts_Js_vel_shldr.Length, ts_Js_pos_elbw.Length, ...
%             ts_Js_pos_shldr.Length, ts_accX_imu.Length, ts_accY_imu.Length, ts_accY_imu.Length ...
%             ts_gyroX_imu.Length, ts_gyroY_imu.Length, ts_gyroZ_imu.Length, ts_Q0_imu.Length , ts_Q1_imu.Length, ts_Q2_imu.Length, ts_Q3_imu.Length]);

len = min([ts_Js_vel_elbw.Length, ts_Js_vel_shldr.Length, ts_Js_pos_elbw.Length, ts_Js_pos_shldr.Length, ts_cur.Length, ts_trq.Length]);

% building the table of all the topics
time = ts_cur.Time(1:len);
current = ts_cur.Data(1:len);
torque = ts_trq.Data(1:len);
% command = ts_cmd.Data(1:len);    
% position = ts_pos.Data(1:len);
% velocity = ts_vel.Data(1:len);
shldr_position = ts_Js_pos_shldr.Data(1:len);
elbw_position = ts_Js_pos_elbw.Data(1:len);
shldr_velocity = ts_Js_vel_shldr.Data(1:len);
elbw_velocity = ts_Js_vel_elbw.Data(1:len);

% imu_accX = squeeze(ts_accX_imu.Data(1,1,1:len));
% imu_accY = squeeze(ts_accY_imu.Data(1,1,1:len));
% imu_accZ = squeeze(ts_accZ_imu.Data(1,1,1:len));
% 
% imu_gyroX = squeeze(ts_gyroX_imu.Data(1,1,1:len));
% imu_gyroY = squeeze(ts_gyroY_imu.Data(1,1,1:len));
% imu_gyroZ = squeeze(ts_gyroZ_imu.Data(1,1,1:len));

% imu_Q0 = squeeze(ts_Q0_imu.Data(1,1,1:len));
% imu_Q1 = squeeze(ts_Q1_imu.Data(1,1,1:len));
% imu_Q2 = squeeze(ts_Q2_imu.Data(1,1,1:len));
% imu_Q3 = squeeze(ts_Q3_imu.Data(1,1,1:len));


% T = table(time, shldr_position, elbw_position, shldr_velocity, elbw_velocity,...
%     imu_accX, imu_accY, imu_accZ, imu_gyroX, imu_gyroY, imu_gyroZ, imu_Q0, imu_Q1, imu_Q2, imu_Q3);      

T = table(time, shldr_position, elbw_position, shldr_velocity, elbw_velocity, current, torque);      

data = T.Variables;
column_name = T.Properties.VariableNames;

save([name(1:end-4),'.mat'],'data', 'column_name');

