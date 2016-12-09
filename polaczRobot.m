t = tcpip('192.168.1.2', 59002, 'NetworkRole', 'client');
fclose(t)
fopen(t)
pause(1)