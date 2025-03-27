close all;

% Import danych z pliku csv i przepisanie danych do wektorow
%x = csvread ('pulse_2.csv');
x = csvread ('puls_6.csv');
Light = x(:,1);
Temp = x(:,2);

%% Sygnal w funkcji czasu
% Obliczednie wekotra czasu w s na podstawie czestotliwosci probkowania i dlugosci wektora danych
Fs = 10;
Time_s = (1/Fs):(1/Fs):(length(Light)/(Fs));

%% PSD - gestosc widmowa mocy
% Dobranie parametrow PSD (gestosci widmowej mocy)
% Ilosc probek PSD, zalezna od ilosci probek sygnalu
nfft = 2^11;
% Dlugosc okna dla metody Welch (metod liczenia PSD)
nwind = 2^8;
% Wspolczynik Beta dla onka Kaisera (5-15 bedzie dobrze)
beta = 10;
% Zakldaka dla okna (musi byc mniej niz dlugosc okna)
% Dla Matalba w probakch np. 128, dla Octacve od 0 do 0.95
overlap = 0.2;
window = kaiser(nwind,beta);
[Pxx,Fp] = pwelch(Light,window,overlap,nfft,Fs);
%(x, window, overlap, Nfft, Fs, range, plot_type, detrend, sloppy)
PsdLight = 10*log10(Pxx);

%% Wykresy
% Narysowanie wektora dnaych
figure("position",[1 1 1920*0.5 1080*0.8]);

sub(1) = subplot(3,1,1);
plot(Time_s,Light);
xlabel('Czas [s]'); ylabel('Napiecie [kod ADC]'); grid on; title('Sygnal w funkcji czasu');

% Narysowanie wektora PSD w funkcji czestotliwosci
sub(1) = subplot(3,1,2);
plot(Fp,PsdLight, 'b');
xlabel('Czestotliwosc [Hz]'); ylabel('Napiecie [dB]'); grid on; title('Sygnal w funkcji czestotliwosci');

% Narysowanie wektora PSD w funkcji tetna
sub(1) = subplot(3,1,3);
plot(Fp*60,PsdLight, 'b');
xlabel('Puls [bpm]'); ylabel('Napiecie [dB]'); grid on; title('Sygnal w funkcji "beats each minute"');