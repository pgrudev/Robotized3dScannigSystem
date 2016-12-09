%% powrot do pozycji zerowej w pionie oraz wyslanie flagi oznaczajacej 
%% obrot pozycjonera o zadany kat
pozycjazerowa
%% 
% znaki='100001';
% x='600000';
% y='000000';
% z='001000';
% w='000000';
% p='000000';
% r='000000';
%% 
flaga='3';  %% oznacza, ze pozycjoner sie obroci
wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
wyslijwiadomosc
flaga='4';