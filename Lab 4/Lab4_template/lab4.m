%% Lab4: Formation Control


%% Setup and initialization
clear
% delete(instrfind); % Legacy serial cleanup, not needed with serialport
% Setup Serial and QTM communication
% Connect to QTM
QCM('connect', '127.0.0.1', '3D');

% Connect to Serial for wireless control
info = serialportlist("available");
if isempty(info)
   error('No ports free!');
end
s = serialport(info(end), 57600, 'DataBits', 8, 'Timeout', 1);
% manual selection
% s = serialport("COM3", 9600, 'DataBits', 8, 'Timeout', 1);
% fopen(s); % Not needed with serialport

% Logfile preparation
thismoment = clock;
date_time = '';
for i = 1:5
    date_time = [date_time, num2str(thismoment(i), '%02d')];
    if i == 3
        date_time = [date_time, '_'];
    end
end

if exist('mission_data', 'dir') ~= 7
    mkdir('mission_data');
end
file_name = ['mission_data/data_',date_time,'.csv'];
fid = fopen(file_name,'w');

% Parameters

% read individual parameters
vhc_param;
disable = 0;
header = 4; % Was 3 originally
command = 65;   % A = 65, (Autonomous)
PWMV = 0;

%% Read QTM data

% Retrieve current position data
labels3d = QCM('3dlabels'); % saves name of tracked markers
num_marker = size(labels3d, 2);
for i = 1:num_marker
    switch string(labels3d(i))
% Reading Wand -----------------------------------------------------------      
        case 'want_target'
            target.index = i;
% Reading Tank 12-----------------------------------------------------
        case 'tank_12_front'
            wr.findex = i;
        case 'tank_12_back'
            wr.bindex = i;
% Reading Tank 11----------------------------------------------------------
        case 'tank_11_front'
            t2.findex = i;
        case 'tank_11_back'
            t2.bindex = i;
% Reading Tank 13----------------------------------------------------------
        case 'tank_13_front'
            t3.findex = i;
        case 'tank_13_back'
            t3.bindex = i;
        otherwise
    end
end

nun_usable_marker = 7;
num_veh = 3;
% Read initial data from QTM
rawData = zeros(nun_usable_marker, 3);
rData = QCM;

wr.front_marker = rData(wr.findex,1:2); wr.back_marker = rData(wr.bindex,1:2); wr.pos_old = (wr.front_marker + wr.back_marker)/2;
t2.front_marker = rData(t2.findex,1:2); t2.back_marker = rData(t2.bindex,1:2); t2.pos_old = (t2.front_marker + t2.back_marker)/2;
t3.front_marker = rData(t3.findex,1:2); t3.back_marker = rData(t3.bindex,1:2); t3.pos_old = (t3.front_marker + t3.back_marker)/2;

time.old = 0;
loop = 1;
tic
while(disable < 1)
%     wr.disable = disable;
    time.curr = toc;  
    time.dt = time.curr - time.old;
    % rData structure: x,y,z,roll,pitch,yaw
    rData = QCM;
    rData_old = rData;
    % recording the wand location
    targetz = rData(target.index, 3);

%     poswr = {rData("index of front marker of WR", 1:2) + rData("index of back marker of WR", 1:2)} / 2;
%     head_vec = [rData("index of front marker of WR", 1:2) - rData("index of front marker of WR", 1:2)];
    for i = 1:num_marker
        % grabbing the front and back positional marker from QCM
        switch string(labels3d(i))
            case {'tank_12_front', 'tank_12_back'}
%                 disp("Found WR!");
                wr.disable = disable;
                wr.front_marker = rData(wr.findex,1:2); 
                wr.back_marker = rData(wr.bindex,1:2);
                wr.heading_vec = wr.front_marker - wr.back_marker;
                wr.pos = (wr.front_marker + wr.back_marker)/2;
                wr.update = 1;
            case {'tank_11_front', 'tank_11_back'}
%                 disp("Found Tank 2!");
                t2.disable = disable;
                t2.front_marker = rData(t2.findex,1:2); 
                t2.back_marker = rData(t2.bindex,1:2); 
                t2.heading_vec = t2.front_marker - t2.back_marker;
                t2.pos = (t2.front_marker + t2.back_marker)/2;
                t2.update = 1;
            case {'tank_13_front', 'tank_13_back'}
%                 disp("Found Tank 3!");
                t3.disable = disable;
                t3.front_marker = rData(t3.findex,1:2); 
                t3.back_marker = rData(t3.bindex,1:2); 
                t3.heading_vec = t3.front_marker - t3.back_marker;
                t3.pos = (t3.front_marker + t3.back_marker)/2;
                t3.update = 1;
            otherwise
        end
    end
   %% formation control for individual rover
   [wr,t2,t3,disable] = formation_control(wr,t2,t3,time,disable);
 
   %--------------
    % Calculate the commands to each rover
    % Then, send command
    if wr.update && disable < 1
        wr.scommand = [header command PWMV wr.PWML wr.PWMR wr.DIRL wr.DIRR wr.ID 0 13];
        %wr.scommand = [header command PWMV 37 37 1 1 wr.ID 0 13]
        checksum = 0;
        for i = 2:1:8
            checksum = bitxor(checksum, wr.scommand(i));
        end
        wr.scommand(9) = checksum;
        % actrual command sent
        write(s, wr.scommand, "uint8");
        wr.scommand = 0;
        if targetz < 500
            disable = disable + 1;
        end
    end
    if t2.update && disable < 1
        t2.scommand = [header command PWMV t2.PWML t2.PWMR t2.DIRL t2.DIRR t2.ID 0 13]
        %t2.scommand = [header command PWMV 72 31 1 1 t2.ID 0 13]

        checksum = 0;
        for i = 2:1:8
            checksum = bitxor(checksum, t2.scommand(i));
        end
        t2.scommand(9) = checksum;
        write(s, t2.scommand, "uint8");
        t2.scommand = 0;
        if targetz < 500
            disable = disable + 1;
        end
    end
    %---------------------
    if t3.update && disable < 1
        t3.scommand = [header command PWMV t3.PWML t3.PWMR t3.DIRL t3.DIRR t3.ID 0 13];
       % t3.scommand = [header command PWMV 37 37 1 1 t3.ID 0 13]
        checksum = 0;
        for i = 2:1:8
            checksum = bitxor(checksum, t3.scommand(i));
        end
        t3.scommand(9) = checksum;
        % actrual command sent
        write(s, t3.scommand, "uint8");
        t3.scommand = 0;
        if targetz < 500
            disable = disable + 1;
        end
    end
    %----------------------
    % checking the wand position/height (mm) from the ground 
    if targetz < 500
        disable = disable + 1;
    end

    %% Logging
    % data saving, more data can be added to here
    % Add data to data_to_log and redefine the first row of csv file
    %----------------------------------------------------------------------
    data_to_log = [double(time.curr), double(wr.pos(1)), double(wr.pos(2)), double(t2.pos(1)), double(t2.pos(2)), double(t3.pos(1)), double(t3.pos(2))];
    if loop == 1
        fprintf(fid, ['time, wr_pos_x[mm], wr_pos_y[mm], t2_pos_x[mm], t2_pos_y[mm], t3_pos_x[mm], t3_pos_y[mm],']);
 %----------------------------------------------------------------------
        format_string = '';
        for i=1:1:length(data_to_log)
            format_string = strcat(format_string, '%3.4f');
            if i < length(data_to_log)
                format_string = strcat(format_string, ',');
            else
                format_string = strcat(format_string, '\n');
            end
        end
    end
    fprintf(fid, format_string, data_to_log);
    loop = loop + 1;
    time.old = time.curr;
end

PWMV = 0; % For blimp
PWML = 0; % Speed (Left)
PWMR = 0; % Speed (Right)
DIRL = 1; % Direction (Left)
DIRR = 1; % Direction (Right)
for roverID = 11:13
    scommand = [header command PWMV PWML PWMR DIRL DIRR roverID 0 13]
    checksum = 0;
    for i = 2:1:8
        checksum = bitxor(checksum, scommand(i));
    end
    scommand(9) = checksum;
    write(s, scommand, "uint8");
    scommand = 0; 
end
% end of file, close all the connections
disp('terminated');
fclose(fid);
clear s;
% delete(instrfind); % Legacy serial cleanup, not needed with serialport
QCM('disconnect');