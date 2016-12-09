% Setup Lidar 
% % Configures Serial Communication and Updates Sensor Communication to
% SCIP2.0 Protocol.
% % Checks Version Information and switches on Laser.
% Author- Shikhar Shrestha, IIT Bhubaneswar
instrreset
lidar=serial('COM21','baudrate',115200);
set(lidar,'Timeout',(1/4));
set(lidar,'InputBufferSize',40000);
set(lidar,'Terminator','LF');
%set(lidar,'Terminator','CR');
fopen(lidar);
pause(0.1);
% % fprintf(lidar,'SCIP2.0');
% % pause(0.1);
% % fscanf(lidar);
% % %%fprintf(lidar,'VV');
% % pause(0.1);
% % %%fscanf(lidar)
fprintf(lidar,'BM');
pause(0.1);
fscanf(lidar);
fscanf(lidar);
