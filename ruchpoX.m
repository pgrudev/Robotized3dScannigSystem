znakX=znaki(1); 
x=str2num(x);
%% sprawdzenie, czy znak z nie jest '-'
if znakX=='1'
    x=-x;
end
%% konwersja 'x' na numer i dodanie do niego dlugosci kroku:      
x=x+krok_x*1000;
%% zapewnienie odpowiedniego znaku po ruchu
if x<0
    znaki(1)='1';
else 
    znaki(1)='0';
end
x=abs(x); %znak uwzgledniony w ramce znakow
%% uzupelnienie zerami do 6 znakow, zamiana na string
x=sprintf('%06d', x);
%% wyslanie wiadomosci
wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
wyslijwiadomosc
licznik_x=licznik_x+1;  