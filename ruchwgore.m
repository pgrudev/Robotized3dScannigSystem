znakZ=znaki(3); 
z=str2num(z);
%% sprawdzenie, czy znak z nie jest '-'
if znakZ=='1'
    z=-z;
end
%% konwersja 'z' na numer i dodanie do niego dlugosci kroku:      
z=z+krok_pomiaru_pion*1000;
%% zapewnienie odpowiedniego znaku po ruchu
if z<0
    znaki(3)='1';
else 
    znaki(3)='0';
end
z=abs(z); %znak uwzgledniony w ramce znakow
%% uzupelnienie zerami do 6 znakow, zamiana na string
z=sprintf('%06d', z);
%% wyslanie wiadomosci
wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
wyslijwiadomosc
licz=licz+1;