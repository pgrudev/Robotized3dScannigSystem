znakP=znaki(5); 
p=str2num(p);
%% sprawdzenie, czy znak z nie jest '-'
if znakP=='1'
    p=-p;
end
%% konwersja 'p' na numer i dodanie do niego dlugosci kroku:      
p=p+krok_pomiaru_kat_p*1000;
%% zapewnienie odpowiedniego znaku po ruchu
if p<0
    znaki(5)='1';
else 
    znaki(5)='0';
end
p=abs(p); %znak uwzgledniony w ramce znakow
%% uzupelnienie zerami do 6 znakow, zamiana na string
p=sprintf('%06d', p);
%% wyslanie wiadomosci
flaga='5';

wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);

pause(0.1);
wyslijwiadomosc
przychodzaca='5';
