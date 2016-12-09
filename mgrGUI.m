function varargout = mgrGUI(varargin)
% MGRGUI MATLAB code for mgrGUI.fig
%      MGRGUI, by itself, creates a new MGRGUI or raises the existing
%      singleton*.
%
%      H = MGRGUI returns the handle to a new MGRGUI or the handle to
%      the existing singleton*.
%
%      MGRGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MGRGUI.M with the given input arguments.
%pcwrite(ptCloud,'object3d.pcd','Encoding','ascii');
%      MGRGUI('Property','Value',...) creates a new MGRGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mgrGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mgrGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mgrGUI

% Last Modified by GUIDE v2.5 12-Sep-2016 13:12:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mgrGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @mgrGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before mgrGUI is made visible.
function mgrGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mgrGUI (see VARARGIN)

clear global dostepne
tekst='autor: Pawe³ Grudzieñ';
set(handles.text7, 'String', tekst);
% Choose default command line output for mgrGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mgrGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mgrGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%% WYSWIETL CHMURE
% --- Executes on button press in pushbutton1. 
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%
global punkt
global wykresdata
global aktualna
global odszumiona skan_gora bez_odszumiania dodatkowa1 dodatkowa2 polaczona1 polaczona2

nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz nazwê chmury:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end

prompt = {'Wielkoœæ markera:'};
dlg_title = 'Podaj wielkoœæ markera';
num_lines = 1;
defaultans = {'55'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
wielkosc_markera=str2double(answer(1));


 figure
 switch s
        case 1
            pcshow(odszumiona,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
             ylabel('X') % x-axis label
             zlabel('N') % x-axis label
        case 2
            pcshow(skan_gora,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
             ylabel('X') % x-axis label
             zlabel('N') % x-axis label
           
        case 3
            pcshow(bez_odszumiania,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
             ylabel('X') % x-axis label
            zlabel('N') % x-axis label
        case 4 
            pcshow(dodatkowa1,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
             ylabel('X') % x-axis label
             zlabel('N') % x-axis label
        case 5 
            pcshow(dodatkowa2,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
             ylabel('X') % x-axis label
             zlabel('N') % x-axis label
        case 6
            pcshow(polaczona1,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
            ylabel('X') % x-axis label
            zlabel('N') % x-axis label
        case 7
            pcshow(polaczona2,'MarkerSize',wielkosc_markera);
            xlabel('Y') % x-axis label
            ylabel('X') % x-axis label
            zlabel('N') % x-axis label
 end
    
 wykresdata=gcf;
 xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 zlabel('N') % x-axis label
 guidata(hObject, handles);
%%

%% Przetwarzanie pliku
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global chmura
global punkt
global denoised
global aktualna
global wysokosc
global dostepne
global odszumiona skan_gora bez_odszumiania dodatkowa1 dodatkowa2 polaczona1 polaczona2

%% status 'Proszê czekaæ'
tekst='Proszê czekaæ';
set(handles.text3, 'String', tekst);

set(handles.figure1, 'pointer', 'watch') 
drawnow;
clear punkt
%% przetwarzanie danych
    [A B]=size(chmura);
    kat_obrotu_pozycjoner_stopnie=chmura(1,1);
    odleglosc_os_Z=chmura(1,4);
    

    for i=2:A
         odleglosc_os_Z=sqrt((chmura(i,6))*(chmura(i,6))+(chmura(i,7))*(chmura(i,7)));
         punkt(i,1)=chmura(i,1)*sin(chmura(i,4)) ;%*cos(-deg2rad(chmura(i,10)));
         punkt(i,2)=odleglosc_os_Z-cos(chmura(i,4))*chmura(i,1) *cos(-deg2rad(chmura(i,10)));
         punkt(i,3)= chmura(i,8)-chmura(i,1)*sin(deg2rad(chmura(i,10)))*cos((chmura(i,4)));
         R=rotz(-(kat_obrotu_pozycjoner_stopnie*chmura(i,3)));
         punkt(i,:)=(R*punkt(i,:)')';
    end
%% usuwanie poni¿ej poziomu 0 - nie u¿ywane
    j=1;
    for i=1:A
        if punkt(i,3)>0;
            punktbezdolu(j,:)=punkt(i,:);
            j=j+1;
        end
    end
%% nadanie chmurze nazwy
     nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz nazwê chmury:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
    
    dostepne=strcat(dostepne,{' '},nazwy{s});

    tekst='dane przetworzone';
    set(handles.text3, 'String', tekst);
    
    tekst2=dostepne;
    set(handles.text6, 'String', tekst2);
    
    
    switch s
        case 1
            odszumiona=pointCloud(punkt);
        case 2
            skan_gora=pointCloud(punkt);
        case 3
            bez_odszumiania=pointCloud(punkt);
        case 4 
            dodatkowa1=pointCloud(punkt);
        case 5 
            dodatkowa2=pointCloud(punkt);
        case 6
            polaczona1=pointCloud(punkt);
        case 7
            polaczona2=pointCloud(punkt);
    end   
  %%  

set(handles.figure1, 'pointer', 'arrow')

%% WCZYTYWANIE PLIKU
% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% wczytywanie
global chmura
tekst='Proszê czekaæ';
set(handles.text3, 'String', tekst);

set(handles.figure1, 'pointer', 'watch') 
    drawnow;
    [file1,path1] = uigetfile({'*.*'},'Wybierz plik z chmur¹');
    %combine the filename and pathname

if file1==0 
       tekst=' ';
set(handles.text3, 'String', tekst);
    
set(handles.figure1, 'pointer', 'arrow')
    return
    
end

    directory=strcat(path1,file1);
    chmura=load(directory);
    tekst='wczytano dane';
set(handles.text3, 'String', tekst);
    
set(handles.figure1, 'pointer', 'arrow')
%%






% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on mouse press over axes background.
function axes2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object deletion, before destroying properties.
function axes2_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end






%% SKANOWANIE
% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global kat_obrotu_pozycjoner
global krok_pomiaru_pion
%% ustawienia 
prompt = {'Podaj nazwê pliku do zapisu skanu:','Podaj rozdzielczoœæ obrotu pozycjonera:','Podaj krok robota w pionie:'};
dlg_title = 'Ustawienia skanowania';
num_lines = 1;
defaultans = {'czaj1tetst','030','3'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
nazwapliku=strjoin(answer(1));
kat_obrotu_pozycjoner=strjoin(answer(2));
krok_pomiaru_pion=str2double(answer(3));
licz=0;

tekst='skanowanie w toku';
set(handles.text3, 'String', tekst);
%% polacz skaner
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
%% polacz robot

 
t = tcpip('192.168.1.2', 59002, 'NetworkRole', 'client');
% fclose(t)
fopen(t)
pause(1)
%% polaczenia

plik=fopen(nazwapliku,'w');
%% deklaracje
calkowity_obrot_pozycjonera=0;
moznaobracac=0;

kat_obrotu_pozycjoner_stopnie=str2num(kat_obrotu_pozycjoner);
numer_obrotu_pozycjoner=0;

przychodzaca='9';
%% pozycja zerowa rozbita
pozycjazerowa
%% wyznaczenie odleglosci czujnika od osi Z
xodl=str2num(x)/1000;
yodl=str2num(y)/1000;
odleglosc_os_Z=sqrt(power(xodl,2)+power(yodl,2));
%% przeslanie pozycji zerowej
wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
wyslijwiadomosc
%% zapisanie parametrow skanowania w pliku pomiarowym
  fprintf(plik,'%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t',kat_obrotu_pozycjoner_stopnie,krok_pomiaru_pion,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z);
  fprintf(plik,'\n');
%% program glowny
while(calkowity_obrot_pozycjonera<=360)
    %% odebranie wiadomosci
    if t.BytesAvailable>0
        przychodzaca=char(fread(t, t.BytesAvailable));;
   end
    %% jesli robot dojechal na miejsce:
     if strcmpi(przychodzaca(1),'1') == 1  
       xdopomiaru=str2num(x)/1000;
       ydopomiaru=str2num(y)/1000;
       zdopomiaru=str2num(z)/1000;
       wdopomiaru=str2num(w)/1000;
       pdopomiaru=str2num(p)/1000;
       rdopomiaru=str2num(r)/1000;
       
     %% sprawdzanie znaków
        if znaki(1)=='1'
           xdopomiaru=-xdopomiaru;
       end
       
       if znaki(2)=='1'
           ydopomiaru=-ydopomiaru;
       end
       
       if znaki(3)=='1'
           zdopomiaru=-zdopomiaru;
       end
       
       if znaki(4)=='1'
           wdopomiaru=-wdopomiaru;
       end
       
       if znaki(5)=='1'
           pdopomiaru=-pdopomiaru;
       end
       
       if znaki(6)=='1'
           rdopomiaru=-rdopomiaru;
       end
       
       
       %% wykonaj pomiar
                                pom=LidarScan2test(lidar);
                            for i=1:1:682
                                if pom(i)>200 && pom(i)<900
                                    n=i-387;
                                    alfa=360/1024*n;
                                    alfa=alfa*pi/180;
                                    fprintf(plik,'%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t',pom(i),zdopomiaru,numer_obrotu_pozycjoner,alfa,i,xdopomiaru,ydopomiaru,zdopomiaru,wdopomiaru,pdopomiaru,rdopomiaru);
                                    fprintf(plik,'\n');
                                else
                                    pom(i)=0;
                                end
                            end

       %% jesli pomiar zerowy == wyjechal ponad krawedz przedmiotu
                    if mean(pom)==0;
                        moznaobracac=1;
                        pause(0.3)
                        przychodzaca='5';
                        
                    end
       %% ruch w gore
        ruchwgore; 
       przychodzaca='5';
    end
    
    %% obrotpozycjonera
    if moznaobracac==1
        moznaobracac=0;
        obrocpozycjoner
        pause(1)
        numer_obrotu_pozycjoner=numer_obrotu_pozycjoner+1;
        pause(1)
        
        pause(1)
        calkowity_obrot_pozycjonera=kat_obrotu_pozycjoner_stopnie*numer_obrotu_pozycjoner;
        pause(1)
        disp('numer obrotu pozycjonera wynosi:')
        disp(numer_obrotu_pozycjoner)
    end       
    
    %% 

%% zamykanie programu jesli otrzymano znak zamkniecia
    if strcmpi(przychodzaca(1),'2') == 1  
        fclose(t)
        calkowity_obrot=99999;
    end 
end
%% rozlaczanie

fprintf(lidar,'QT');
fclose(lidar);
fclose(t);
fclose(plik);
%%



% % % % --- Executes on button press in pushbutton12.
% % % function pushbutton12_Callback(hObject, eventdata, handles)
% % % % hObject    handle to pushbutton12 (see GCBO)
% % % % eventdata  reserved - to be defined in a future version of MATLAB
% % % % handles    structure with handles and user data (see GUIDATA)
% % % global nazwapliku
% % % global kat_obrotu_pozycjoner
% % % global krok_pomiaru_pion
% % % 
% % % prompt = {'Podaj nazwê pliku do zapisu skanu:','Podaj rozdzielczoœæ obrotu pozycjonera:','Podaj krok robota w pionie:'};
% % % dlg_title = 'Ustawienia skanowania';
% % % num_lines = 1;
% % % defaultans = {'czaj1tetst','030','3'};
% % % answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
% % % nazwapliku=strjoin(answer(1));
% % % kat_obrotu_pozycjoner=strjoin(answer(2));
% % % krok_pomiaru_pion=str2double(answer(3));


% % % % --- Executes on button press in pushbutton19.
% % % function pushbutton19_Callback(hObject, eventdata, handles)
% % % % hObject    handle to pushbutton19 (see GCBO)
% % % % eventdata  reserved - to be defined in a future version of MATLAB
% % % % handles    structure with handles and user data (see GUIDATA)
% % % global t 
% % % t = tcpip('192.168.1.2', 59002, 'NetworkRole', 'client');
% % % % fclose(t)
% % % fopen(t)
% % % pause(1)
% % % tekst='Po³¹czono';
% % % set(handles.text4, 'String', tekst);
% % % 

% % % 
% % % % --- Executes on button press in pushbutton20.
% % % function pushbutton20_Callback(hObject, eventdata, handles)
% % % % hObject    handle to pushbutton20 (see GCBO)
% % % % eventdata  reserved - to be defined in a future version of MATLAB
% % % % handles    structure with handles and user data (see GUIDATA)
% % % % Setup Lidar 
% % % % % Configures Serial Communication and Updates Sensor Communication to
% % % % SCIP2.0 Protocol.
% % % % % Checks Version Information and switches on Laser.
% % % % Author- Shikhar Shrestha, IIT Bhubaneswar
% % % instrreset
% % % global lidar
% % % lidar=serial('COM21','baudrate',115200);
% % % set(lidar,'Timeout',(1/4));
% % % set(lidar,'InputBufferSize',40000);
% % % set(lidar,'Terminator','LF');
% % % %set(lidar,'Terminator','CR');
% % % fopen(lidar);
% % % pause(0.1);
% % % % % fprintf(lidar,'SCIP2.0');
% % % % % pause(0.1);
% % % % % fscanf(lidar);
% % % % % %%fprintf(lidar,'VV');
% % % % % pause(0.1);
% % % % % %%fscanf(lidar)
% % % fprintf(lidar,'BM');
% % % pause(0.1);
% % % fscanf(lidar);
% % % fscanf(lidar);
% % % tekst='Po³¹czono';
% % % set(handles.text5, 'String', tekst);



% % % % % --- Executes on button press in pushbutton22.
% % % % function pushbutton22_Callback(hObject, eventdata, handles)
% % % % % hObject    handle to pushbutton22 (see GCBO)
% % % % % eventdata  reserved - to be defined in a future version of MATLAB
% % % % % handles    structure with handles and user data (see GUIDATA)
% % % % global punkt1 punkt2
% % % % 
% % % % prompt = {'x1','y1','z1','x2','y2','z2'};
% % % % dlg_title = 'Wspó³rzêdne dwóch punktów';
% % % % num_lines = 1;
% % % % defaultans = {'','','','','',''};
% % % % answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
% % % % x1=str2double(answer(1));
% % % % y1=str2double(answer(2));
% % % % z1=str2double(answer(3));
% % % % 
% % % % x2=str2double(answer(4));
% % % % y2=str2double(answer(5));
% % % % z2=str2double(answer(6));
% % % % punkt1=[x1 y1 z1];
% % % % punkt2=[x2 y2 z2];


%% ODSZUMIANIE
% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% ustawienia
global punkt
global denoised
global NumNeighbors
global Threshold
global cloud
global wysokosc
global aktualna
global dostepne
global odszumiona skan_gora bez_odszumiania dodatkowa1 dodatkowa2 polaczona1 polaczona2

nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Co odszumiæ?',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
 switch s
        case 1
            denoised=(odszumiona);
        case 2
            denoised=(skan_gora);
           
        case 3
            denoised=(bez_odszumiania);
        case 4 
            denoised=(dodatkowa1);
        case 5 
            denoised=(dodatkowa2);
        case 6
            denoised=(polaczona1);
        case 7
            denoised=(polaczona2);
 end

tekst='Proszê czekaæ';
set(handles.text3, 'String', tekst);
set(handles.figure1, 'pointer', 'watch') 
    drawnow;
%% odszumianie 1.
prompt = {'Podaj iloœæ s¹siadów:','Podaj rozdzielczoœæ:'};
dlg_title = 'Ustawienia odszumiania';
num_lines = 1;
defaultans = {'800','0.15'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);

NumNeighbors=str2num(strjoin(answer(1)));
Threshold=str2num(strjoin(answer(2)));



set(handles.figure1, 'pointer', 'watch') 
drawnow;

den=pcdenoise(denoised,'NumNeighbors',NumNeighbors,'Threshold',Threshold);
%% odszumianie 2.
prompt = {'Podaj iloœæ s¹siadów:','Podaj rozdzielczoœæ:'};
dlg_title = 'Ustawienia odszumiania';
num_lines = 1;
defaultans = {'500','0.9'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
Threshold=str2num(strjoin(answer(2)));
NumNeighbors=str2num(strjoin(answer(1)));


denoised=pcdenoise(den,'NumNeighbors',NumNeighbors,'Threshold',Threshold);

figure;
pcshow(denoised);
tekst='dane odszumione';
xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 zlabel('N') % x-axis label
set(handles.text3, 'String', tekst);

set(handles.figure1, 'pointer', 'arrow')
aktualna=denoised;
wysokosc=max(denoised.Location(:,3));
%% zapis chmury
 nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz nazwê chmury:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
   
    dostepne=strcat(dostepne,{' '},nazwy{s});

    tekst='dane przetworzone';
    set(handles.text3, 'String', tekst);
    
    tekst2=dostepne;
    set(handles.text6, 'String', tekst2);
    
    
    switch s
        case 1
            odszumiona=denoised;
        case 2
            skan_gora=denoised;
        case 3
            bez_odszumiania=denoised;
        case 4 
            dodatkowa1=denoised;
        case 5 
            dodatkowa2=denoised;
        case 6
            polaczona1=denoised;
        case 7
            polaczona2=denoised;
    end
    set(handles.figure1, 'pointer', 'arrow') 
    %%
%%

% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%SKANUJ GÓRÊ
% --- Executes on button press in pushbutton30.
function pushbutton30_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% SKANUJ GORE 
global wysokosc
if isempty(wysokosc)==1
    ed = errordlg('Brak danych o wysokoœci obiektu','Error');
    set(ed, 'WindowStyle', 'modal');
    uiwait(ed);
    choice = questdlg('Czy chcesz podaæ wysokoœæ rêcznie?');
    switch choice
        case 'No'
             return
        case 'Yes'
            prompt = {'Podaj wysokoœæ obiektu:'};
            dlg_title = 'Wysokosc obiektu';
            num_lines = 1;
            defaultans = {'200'};
            answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
            wysokosc=str2double(answer(1));
        case 'Cancel'
            return
    end
end
    



prompt = {'Podaj nazwê pliku do zapisu skanu:','Podaj krok robota w poziomie:','Podaj wspó³rzêdn¹ startu oœ X:'};
dlg_title = 'Ustawienia skanowania góry';
num_lines = 1;
defaultans = {'czaj1tetst','1','450'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
nazwapliku=strjoin(answer(1));
krok_pomiaru_poziom=str2double(answer(2));
start=str2double(answer(3));
start=start*1000;

tekst='skanowanie w toku';
set(handles.text3, 'String', tekst);
%% polaczenia
polaczSkaner
polaczRobot
plik=fopen(nazwapliku,'w');
%% deklaracje
calkowity_obrot_pozycjonera=0;
moznaobracac=0;
kat_obrotu_pozycjoner='298';
kat_obrotu_pozycjoner_stopnie=str2num(kat_obrotu_pozycjoner);
numer_obrotu_pozycjoner=0;
krok_pomiaru_pion=40;
krok_pomiaru_kat_p=1;
przychodzaca='9';
licznik_kata=0;
wys_obiektu=wysokosc;
fl=1;
krok_x=krok_pomiaru_poziom;
licznik_x=0;
moznajechac=0;
%% pozycja zerowa rozbita
znaki='100000';
x=sprintf('%06d', start);
% x='450000';
y='000000';
z=num2str((wysokosc+130)*1000);
w='000000';
p='040000';
r='000000';
flaga='0';
licz=0;
%% wyznaczenie odleglosci czujnika od osi Z
xodl=str2num(x)/1000;
yodl=str2num(y)/1000;
odleglosc_os_Z=sqrt(power(xodl,2)+power(yodl,2));
%% przeslanie pozycji zerowej
wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
wyslijwiadomosc
%% zapisanie parametrow skanowania w pliku pomiarowym
  fprintf(plik,'%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t',kat_obrotu_pozycjoner_stopnie,krok_pomiaru_poziom,krok_pomiaru_kat_p,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z);
  fprintf(plik,'\n');
%% program glowny
while(licznik_x<(340/krok_x))
    licznik_x;
    %% odebranie wiadomosci
    if t.BytesAvailable>0
        przychodzaca=char(fread(t, t.BytesAvailable));
   end
    %% jesli robot dojechal na miejsce:
     if strcmpi(przychodzaca(1),'1') == 1     
         %% xyzwpr do pomiaru
       xdopomiaru=str2num(x)/1000;
       ydopomiaru=str2num(y)/1000;
       zdopomiaru=str2num(z)/1000;
       wdopomiaru=str2num(w)/1000;
       pdopomiaru=str2num(p)/1000;
       rdopomiaru=str2num(r)/1000;      
         %% sprawdzanie znaków
        if znaki(1)=='1'
           xdopomiaru=-xdopomiaru;
       end
       
       if znaki(2)=='1'
           ydopomiaru=-ydopomiaru;
       end
       
       if znaki(3)=='1'
           zdopomiaru=-zdopomiaru;
       end
       
       if znaki(4)=='1'
           wdopomiaru=-wdopomiaru;
       end
       
       if znaki(5)=='1'
           pdopomiaru=-pdopomiaru;
       end
       
       if znaki(6)=='1'
           rdopomiaru=-rdopomiaru;
       end     
         %% wykonaj pomiar
                                pom=LidarScan2test(lidar);
                            for i=1:1:682
                                if pom(i)>5 && pom(i)<900
                                    n=i-387;
                                    alfa=360/1024*n;
                                    alfa=alfa*pi/180;
                                    fprintf(plik,'%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t',pom(i),zdopomiaru,numer_obrotu_pozycjoner,alfa,i,xdopomiaru,ydopomiaru,zdopomiaru,wdopomiaru,pdopomiaru,rdopomiaru);
                                    fprintf(plik,'\n');
                                else
                                    pom(i)=0;
                                end
                            end
        
         moznajechac=1;
     end
     
     
    

     %% ruch po osi X
     if moznajechac==1;
         moznajechac=0;
      ruchpoX
      przychodzaca='5';
     end       
        
    
          

%% zamykanie programu jesli otrzymano znak zamkniecia
    if strcmpi(przychodzaca(1),'2') == 1  
        fclose(t)
        krok_x=99999999;
    end 
end
%% rozlaczanie

fprintf(lidar,'QT');
fclose(lidar);
fclose(t);
fclose(plik);
%%


%% DOSKANOWANIE
% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% DOSKANOWANIE

global kat_pozycjoner_doskanowanie
global wysokosc
 
%% brak wysokosci
if isempty(wysokosc)==1
    ed = errordlg('Brak danych o wysokoœci obiektu','Error');
    set(ed, 'WindowStyle', 'modal');
    uiwait(ed);
    choice = questdlg('Czy chcesz podaæ wysokoœæ rêcznie?');
    switch choice
        case 'No'
             return
        case 'Yes'
            prompt = {'Podaj wysokoœæ obiektu:'};
            dlg_title = 'Wysokosc obiektu';
            num_lines = 1;
            defaultans = {'200'};
            answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
            wysokosc=str2double(answer(1));
        case 'Cancel'
            return
    end
end
%% brak kata doskanowania
if isempty(kat_pozycjoner_doskanowanie)==1
    ed = errordlg('Brak danych o miejscu doskanowania','Error');
    set(ed, 'WindowStyle', 'modal');
    uiwait(ed);
    choice = questdlg('Czy chcesz podaæ wspó³rzêdne punktu do doskanowania?');
    switch choice
        case 'No'
             return
        case 'Yes'
            prompt = {'x:','y:','z:'};
            dlg_title = 'Wspó³rzêdne';
            num_lines = 1;
            defaultans = {'','',''};
            answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
            x_c=str2double(answer(1));
            y_c=str2double(answer(2));
            z_c=str2double(answer(3));
            kat_pozycjoner_doskanowanie_stopnie=atand(y_c/x_c);
           
            %% kat obrotu pozycjonera
wybor_x=C(x_c)
wybor_y=C(y_c)
kat=wybor_x/wybor_y

if wybor_x>0 && wybor_y>0
    kat_pozycjoner_doskanowanie=atand(kat);
    kat_pozycjoner_doskanowanie=360-kat_pozycjoner_doskanowanie;
end


if wybor_x>0 && wybor_y<0
kat_pozycjoner_doskanowanie=atand(kat)+180;
kat_pozycjoner_doskanowanie=360-kat_pozycjoner_doskanowanie;
end

if wybor_x<0 && wybor_y>0
    kat_pozycjoner_doskanowanie=atand(kat);
    kat_pozycjoner_doskanowanie=abs(kat_pozycjoner_doskanowanie);
end

if wybor_x<0 && wybor_y<0
    
 kat_pozycjoner_doskanowanie=atand(kat)+180;
  kat_pozycjoner_doskanowanie=360- kat_pozycjoner_doskanowanie;
end


        case 'Cancel'
            return
    end
end
%% ustawienia skanowania


prompt = {'Podaj nazwê pliku do zapisu skanu:','Podaj rozdzielczoœæ kata p:','Podaj krok w pionie:'};
dlg_title = 'Ustawienia skanowania';
num_lines = 1;

proponowany_krok_pion=num2str(round((wysokosc)/6));
defaultans = {'czaj1tetst','0.5',proponowany_krok_pion};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
nazwapliku=strjoin(answer(1));
krok_pomiaru_kat_p=str2double(answer(2));
krok_pomiaru_pion=str2double(answer(3));

tekst='skanowanie w toku';
set(handles.text3, 'String', tekst);
wys_obiektu=wysokosc+2*krok_pomiaru_pion;
wys=round(wysokosc/krok_pomiaru_pion)+2;
licz=0;
polaczSkaner
polaczRobot
plik=fopen(nazwapliku,'w');
skok=20/krok_pomiaru_kat_p;
%% deklaracje
calkowity_obrot_pozycjonera=0;
moznaobracac=1;
kat_pozycjoner_doskanowanie=round(kat_pozycjoner_doskanowanie);
kat_obrotu_pozycjoner=sprintf('%03d', kat_pozycjoner_doskanowanie);

kat_obrotu_pozycjoner_stopnie=str2num(kat_obrotu_pozycjoner);
numer_obrotu_pozycjoner=0;


przychodzaca='9';
licznik_kata=0;

obr=0;
fl=1;


%% pozycja zerowa rozbita
pozycjazerowa
%% wyznaczenie odleglosci czujnika od osi Z
xodl=str2num(x)/1000;
yodl=str2num(y)/1000;
odleglosc_os_Z=sqrt(power(xodl,2)+power(yodl,2));

%% przeslanie pozycji zerowej
wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
wyslijwiadomosc

%% zapisanie parametrow skanowania w pliku pomiarowym
  fprintf(plik,'%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t',kat_obrotu_pozycjoner_stopnie,krok_pomiaru_pion,krok_pomiaru_kat_p,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z,odleglosc_os_Z);
  fprintf(plik,'\n');

%% program glowny
 while(str2num(z)/1000<=wys_obiektu)
% while licz<=wys
    %% odebranie wiadomosci
    if t.BytesAvailable>0
        przychodzaca=char(fread(t, t.BytesAvailable));
   end
    %% jesli robot dojechal na miejsce:
     if ((strcmpi(przychodzaca(1),'1') == 1)  || (obr==1))
         obr=0;
         
         
         %% xyzwpr do pomiaru
       xdopomiaru=str2num(x)/1000;
       ydopomiaru=str2num(y)/1000;
       zdopomiaru=str2num(z)/1000;
       wdopomiaru=str2num(w)/1000;
       pdopomiaru=str2num(p)/1000;
       rdopomiaru=str2num(r)/1000;
       
     %% sprawdzanie znaków
        if znaki(1)=='1'
           xdopomiaru=-xdopomiaru;
       end
       
       if znaki(2)=='1'
           ydopomiaru=-ydopomiaru;
       end
       
       if znaki(3)=='1'
           zdopomiaru=-zdopomiaru;
       end
       
       if znaki(4)=='1'
           wdopomiaru=-wdopomiaru;
       end
       
       if znaki(5)=='1'
           pdopomiaru=-pdopomiaru;
       end
       
       if znaki(6)=='1'
           rdopomiaru=-rdopomiaru;
       end
       
       
       %% wykonaj pomiar
                                pom=LidarScan2test(lidar);
                            for i=1:1:682
                                if pom(i)>100 && pom(i)<900
                                    n=i-387;
                                    alfa=360/1024*n;
                                    alfa=alfa*pi/180;
                                    fprintf(plik,'%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t,%f\t',pom(i),zdopomiaru,numer_obrotu_pozycjoner,alfa,i,xdopomiaru,ydopomiaru,zdopomiaru,wdopomiaru,pdopomiaru,rdopomiaru);
                                    fprintf(plik,'\n');
                                else
                                    pom(i)=0;
                                end
                            end


     end

%% 
          if fl==1
            p='000000';
            wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
            wyslijwiadomosc
            przychodzaca='5';
            fl=0;
            pause(0.3)
          end

        if licznik_kata <skok
            inkrementacjakata
            przychodzaca='5';
            licznik_kata=licznik_kata+1;
        end

        if licznik_kata == skok
            p='000000';
            wiadomosc=strcat(znaki,x,y,z,w,p,r,flaga,kat_obrotu_pozycjoner);
            wyslijwiadomosc
            licznik_kata=licznik_kata+1;
        end




        if licznik_kata>skok && licznik_kata<2*skok 

            inkrementacjakataminus
            przychodzaca='5';
            licznik_kata=licznik_kata+1;
        end

        if licznik_kata==2*skok;
            licznik_kata=0;
            ruchwgore;
            fl=1;
            przychodzaca='5';
        end
        
        
          if moznaobracac==1
              pause(22)
            moznaobracac=0;
            obrocpozycjoner
            pause(1)
            numer_obrotu_pozycjoner=numer_obrotu_pozycjoner+1;
            pause(1)

            pause(1)
            calkowity_obrot_pozycjonera=kat_obrotu_pozycjoner_stopnie*numer_obrotu_pozycjoner;
            pause(1)
            disp('numer obrotu pozycjonera wynosi:')
            disp(numer_obrotu_pozycjoner)
            pause(25)
              obr=1;
            pause(1)
        end       
        pause(0.3)
        str2num(z)/1000
    %% 

%% zamykanie programu jesli otrzymano znak zamkniecia
    if strcmpi(przychodzaca(1),'2') == 1  
        fclose(t)
        z='99999999';
    end 
end

%% rozlaczanie

fprintf(lidar,'QT');
fclose(lidar);
fclose(t);
fclose(plik);







%% punkty potencjalnej krawedzi
% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global denoised
global potencjalni
global den3


tekst='Proszê czekaæ';
set(handles.text3, 'String', tekst);

set(handles.figure1, 'pointer', 'watch') 
    drawnow;


%wstêpne wytypowanie punktów potencjalnej niech¹g³oœci powierzchni badaj¹c 
%k¹ty pomiêdzy wektorami pomiêdzy s¹siednimi punktami na przekrojach
%poziomych odszumionej chmury punktów


% 
% 
% %% wczytanie danych
% load ('chmuradobadaniaciaglosic.mat');


den3=pointCloud(denoised.Location);
%% Remove Redundant Points from Point Cloud
gridStep = 0.01;
ptCloudOut = pcdownsample(den3,'gridAverage',gridStep)

punkty=ptCloudOut.Location;
j=1;

%% dodanie indeksu do kazdego punktu
for i=1:size(punkty)
punkty(i,4)=j;
j=j+1;
end
posortowane=sortrows(punkty,3);
%% deklaracja pustego uk³adu 50000 macierzy dowolnego rozmiaru
for i=1:50000
    macierztymczasowa{i}=[];
end
k=1;
z=posortowane(1,3);
%% zapisanie punktów z poszczególnych poziomów w uk³adzie macierzy - A
for i=1:size(posortowane)
    
    if posortowane(i,3)==z;
        macierztymczasowa{k}=cat(1,macierztymczasowa{k},posortowane(i,:));
    else
        z=posortowane(i,3);
        k=k+1;
    end
end

%%  usuniecie pustych elementow A
bezpustych=macierztymczasowa(~cellfun('isempty',macierztymczasowa))  ;
rozmiarA=size(bezpustych);
clear macierztymczasowa
potencjalni=[];
potencjalniIDX=[];
potencjalniTemp=[];

%% analiza punktow
%-------------------------------------
for w=1:rozmiarA(2) %wszystkie poziomy
    
    clear  bezpustych1 bezpustych2 tymczas  
    jest_sasiad=0;
    
    tymczas=bezpustych{w};  %przypisanie punktow z danego poziomu do zmiennej tymczas
    bezpustych1=tymczas(:,1:3); %zapisanie samych punktow do dwoch zmiennych
    bezpustych2=tymczas(:,1:3);
    ilosc_sasiadow_max=5;
    [IDX,D] = knnsearch(bezpustych1,bezpustych2,'k',ilosc_sasiadow_max);  %znalezienie sasiadow na poziomie dla wszystkich punktow
    q=1;    %inicjacja zmiennej 
    rozmiarD=size(D);
    for i=1:size(D)   %jeden poziom
        clear badany sasiad katy 
        sasiad=[];
        badany=bezpustych1(IDX(i,1),:); %dla ka¿dego punktu na poziomie w
        ii=1; %inicjacja zmiennej licznika ilosci katow
        for j=2:rozmiarD(2) %dla wszystkich sasiadow - pierwszy pomijany, bo to on sam
             sasiad{j-1}=bezpustych1(IDX(i,j),:);
        end
        
        
        
         if isempty(sasiad)==0
            rozmiar_sasiad=size(sasiad);

            %zapisanie kata pomiedzy kazdymi wektorami
            for j=1:rozmiar_sasiad(2)
                u=badany-sasiad{j};
                for ib=1:rozmiar_sasiad(2)
                    v=badany-sasiad{ib};
                   kat=acosd(dot(u,v)/(norm(u)*norm(v)));
                   if kat>33 || kat<-33

                       jest_sasiad=1;
                   end
                end
            end
            if jest_sasiad==0
                potencjalniTemp(q,:)=badany;
                q=q+1;
            else
                jest_sasiad=0;
            end
         end 
    end
    potencjalni=cat(1,potencjalni,potencjalniTemp);
end
 poten=pointCloud(potencjalni);

 
 %% Remove Redundant Points from Point Cloud
gridStep = 0.01;
ptCloudOut = pcdownsample(poten,'gridAverage',gridStep)
ilosc_potencjalnych_punktow_krawedzi=size(ptCloudOut.Location)

figure
pcshowpair(den3,ptCloudOut,'MarkerSize',100)
xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 zlabel('N') % x-axis label
 
poten=ptCloudOut;
potencjalni=poten.Location;

  tekst='Dane przetworzone';
set(handles.text3, 'String', tekst);
    
set(handles.figure1, 'pointer', 'arrow')







%% punkty do doskanowania
% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global potencjalni
global C
global kat_pozycjoner_doskanowanie
global den3
global denoised
global spodziewane


tekst='Proszê czekaæ';
set(handles.text3, 'String', tekst);

set(handles.figure1, 'pointer', 'watch') 
    drawnow;

den3=pointCloud(denoised.Location);
prompt = {'Podaj spodziewan¹ iloœæ doskanowañ:'};
dlg_title = '';
num_lines = 1;
defaultans = {'3'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
spodziewane=str2double(answer(1));



%u¿ytkownik wybiera ile punktow do doskanowania bedzie (?)
clear potencjalni2

%zmniejszenie liczby punktow potencjalnej nieciaglosci powierzchni

% parametry do dobrania
[IDX,D] = rangesearch(potencjalni,potencjalni,7); 
j=1;
for i=1:size(IDX)
    rozmiar=size(IDX{i});
    if rozmiar(2)>2
        potencjalni2(j,:)=potencjalni(i,:);
        j=j+1;
    end
end
poten2=pointCloud(potencjalni2);
figure
pcshowpair(poten2,den3,'MarkerSize',55)
xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 zlabel('N') % x-axis label

figure
scatter(potencjalni2(:,1),potencjalni2(:,2))
xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 
%wyznaczenie centroidy
potencjalni3=[potencjalni(:,1) potencjalni(:,2)];

[idx,C] = kmeans(potencjalni2,spodziewane);
% for i=1:size(C)
%     C(i,3)=0;
% end
figure
pcshowpair(pointCloud(C),den3,'MarkerSize',100)
xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 zlabel('N') % x-axis label
hold on
plotv(C')
for i=1:spodziewane
    text(C(i,1),C(i,2),C(i,3),num2str(i));
end
srodek=mean(C);
plotv(srodek')
% text(srodek(1,1),srodek(1,2),srodek(1,3),'srodek');


tekst='Dane przetworzone';
set(handles.text3, 'String', tekst);
    
set(handles.figure1, 'pointer', 'arrow')




function pushbutton13_CreateFcn(hObject, eventdata, handles)
function text3_CreateFcn(hObject, eventdata, handles)
function figure1_DeleteFcn(hObject, eventdata, handles)




%POLACZ CHMURY
% --- Executes on button press in pushbutton35.
function pushbutton35_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dostepne
global odszumiona skan_gora bez_odszumiania dodatkowa1 dodatkowa2 polaczona1 polaczona2
%% pierwsza chmura
nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz chmurê nr 1:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
 switch s
        case 1
           chmura1=odszumiona;
        case 2
            chmura1=(skan_gora);
           
        case 3
            chmura1=(bez_odszumiania);
        case 4 
             chmura1=(dodatkowa1);
        case 5 
             chmura1=(dodatkowa2);
        case 6
             chmura1=(polaczona1);
        case 7
             chmura1=(polaczona2);
 end
 %% druga chmura
 nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz chmurê nr 2:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
 switch s
        case 1
           chmura2=odszumiona;
        case 2
            chmura2=(skan_gora);
           
        case 3
            chmura2=(bez_odszumiania);
        case 4 
             chmura2=(dodatkowa1);
        case 5 
             chmura2=(dodatkowa2);
        case 6
             chmura2=(polaczona1);
        case 7
             chmura2=(polaczona2);
 end
 %% polaczenie chmur
 mergerd=pcmerge(chmura1,chmura2,1);
 %% nadanie nazwy
   nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz nazwê chmury:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
   
    dostepne=strcat(dostepne,{' '},nazwy{s});

    tekst='dane przetworzone';
    set(handles.text3, 'String', tekst);
    
    tekst2=dostepne;
    set(handles.text6, 'String', tekst2);
    
    
    switch s
        case 1
            odszumiona=mergerd;
        case 2
            skan_gora=mergerd;
        case 3
            bez_odszumiania=mergerd;
        case 4 
            dodatkowa1=mergerd;
        case 5 
            dodatkowa2=mergerd;
        case 6
            polaczona1=mergerd;
        case 7
            polaczona2=mergerd;
    end
       %% 
       
       
% --- Executes on button press in pushbutton36.
function pushbutton36_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function pushbutton1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton37.
function pushbutton37_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton37 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%WYŒWETL DWIE CHMURY
% --- Executes on button press in pushbutton34.
	
% hObject    handle to pushbutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global dostepne
global odszumiona skan_gora bez_odszumiania dodatkowa1 dodatkowa2 polaczona1 polaczona2
%% pierwsza chmura
nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz chmurê nr 1:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
 switch s
        case 1
           chmura1=odszumiona;
        case 2
            chmura1=(skan_gora);
           
        case 3
            chmura1=(bez_odszumiania);
        case 4 
             chmura1=(dodatkowa1);
        case 5 
             chmura1=(dodatkowa2);
        case 6
             chmura1=(polaczona1);
        case 7
             chmura1=(polaczona2);
 end
 %% druga chmura
 nazwy = {'odszumiona' 'skan_gora' 'bez_odszumiania' 'dodatkowa1' 'dodatkowa2' 'polaczona1' 'polaczona2'};
    [s,v] = listdlg('PromptString','Wybierz chmurê nr 2:',...
                'SelectionMode','single',...
                'ListString',nazwy);
if v==0
    return
end
 switch s
        case 1
           chmura2=odszumiona;
        case 2
            chmura2=(skan_gora);
           
        case 3
            chmura2=(bez_odszumiania);
        case 4 
             chmura2=(dodatkowa1);
        case 5 
             chmura2=(dodatkowa2);
        case 6
             chmura2=(polaczona1);
        case 7
             chmura2=(polaczona2);
 end
 %% wielkosc markera 
prompt = {'Wielkoœæ markera:'};
dlg_title = 'Podaj wielkoœæ markera';
num_lines = 1;
defaultans = {'55'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
wielkosc_markera=str2double(answer(1));
 %% wyswietlanie
 figure
 pcshowpair(chmura1,chmura2,'MarkerSize',wielkosc_markera);
 xlabel('Y') % x-axis label
 ylabel('X') % x-axis label
 zlabel('N') % x-axis label
 %%
 


% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global C
global spodziewane
global kat_pozycjoner_doskanowanie

prompt = {'Podaj numer punktu do doskanowania:'};

dlg_title = 'Nr punktu';
num_lines = 1;
defaultans = {'1'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans);
wybor=str2double(answer(1));


    
if isempty(answer)==1
    return
end
if wybor>spodziewane
    ed = errordlg('Wybra³eœ nieprawid³ow¹ liczbê, wyznacz punkty ponownie \n maksymalny numer to:%f',spodziewane,'Error');
    set(ed, 'WindowStyle', 'modal');
    uiwait(ed);
    return
end

%% kat obrotu pozycjonera
wybor_x=C(wybor,1);
wybor_y=C(wybor,2);
kat=wybor_x/wybor_y;

if wybor_x>0 && wybor_y>0
    kat_pozycjoner_doskanowanie=atand(kat);
%     kat_pozycjoner_doskanowanie=360-kat_pozycjoner_doskanowanie;
end


if wybor_x>0 && wybor_y<0
kat_pozycjoner_doskanowanie=atand(kat)+180;
kat_pozycjoner_doskanowanie=360-kat_pozycjoner_doskanowanie;
end

if wybor_x<0 && wybor_y>0
    kat_pozycjoner_doskanowanie=atand(kat);
    kat_pozycjoner_doskanowanie=abs(kat_pozycjoner_doskanowanie);
end

if wybor_x<0 && wybor_y<0
    
 kat_pozycjoner_doskanowanie=atand(kat)+180;
  kat_pozycjoner_doskanowanie=360- kat_pozycjoner_doskanowanie;
end
kat_pozycjoner_doskanowanie


%%

% --- Executes during object creation, after setting all properties.
function pushbutton31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton42.
function pushbutton42_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear dostepne
clear odszumiona skan_gora bez_odszumiania dodatkowa1 dodatkowa2 polaczona1 polaczona2

 tekst2='';
    set(handles.text6, 'String', tekst2);
