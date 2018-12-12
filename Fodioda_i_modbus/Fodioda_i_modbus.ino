#include <EtherCard.h>
#include <Modbus.h>
#include <ModbusIP_ENC28J60.h>
#include <arduinoFFT.h>

#define SAMPLES 32         //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
#define OdczytFotodiody A0
#define wielkoscTab 500

arduinoFFT FFT = arduinoFFT();

unsigned int i,cod=0/*Ilosc potorzen pentli*/,tab[wielkoscTab],srednia, maks, mini, okres, czestotliwosc=500/*f lasera w Hz*/, sampling_period_us;
bool pomiar=0,zegar=0,znacznikCzasu=0;
int seria=0, czasZegara, Konfiguracja = 0; //0 - Odbiciowa Amplitudowa, 1 - Odbiciowa Czestotliwosciowa, 2 - Transmisyjna
unsigned long startT,czas,sredniaSuma,stoperStart,stoperStop,Tpomiaru, zliczenie=0, microseconds;
float amp=0.3;//wzmocnienie progu aktywacji
double vReal[SAMPLES],vImag[SAMPLES];

//Modbus Registers Offsets (0-9999)
const int SENSOR_IREG = 100;

//ModbusIP object
ModbusIP mb;


void setup() {
    //The media access control (ethernet hardware) address for the shield
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
    //The IP address for the shield
    byte ip[] = { 192, 168, 0, 24 };
    //Config Modbus IP
    mb.config(mac, ip);

    //Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addIreg(SENSOR_IREG);

    //Uruchamiam zegar do pomiaru czasu działania programu
    startT = millis();
    //Ustawiam port szeregowy
    Serial.begin(115200);

    //Parametry początkowe pracy
    Serial.println("");
    Serial.print("Częstotliwość: ");
    Serial.println(czestotliwosc);
    okres=1000/czestotliwosc;
    Serial.print("1 Okres: ");
    Serial.println(okres);
    Serial.println("Zaczynam");

    //Program zaczyna pracę w jednej z konfiguracji //0 - Odbiciowa Amplitudowa, 1 - Odbiciowa Czestotliwosciowa, 2 - Transmisyjna
    switch (Konfiguracja) {
        case 0:
        Serial.println("Konfiguracja Odbiciowa AMPLITUDOWA");

        //WERSJA 1 - ANALIZA AMPLITUDOWA
        //Uruchamianie pętli programu
        while(true){
            mb.task();
            i = analogRead(OdczytFotodiody);//Odczyt z wejścia analogowego
            Serial.print(i);

            if(cod > wielkoscTab && i>(srednia+amp*srednia) && zegar==0){//po wypełnieniu tabeli danymi, pierwszy warunak na rozpoznanie pomiaru = wykrycie wartości większej
                stoperStart=millis();
                Tpomiaru=millis();
                zegar=1;
            }

            Serial.print("\t");
            Serial.print((srednia+amp*srednia)); //wykreslenie progu aktywacji
            Serial.print("\t");
            Serial.print(pomiar*100);//wyznaczenie czy pomiar trwa na wykresie

            if(zegar==1){
                czasZegara= millis()-stoperStart;
                if(czasZegara <= okres*5){
                    znacznikCzasu=1;
                }
                else {
                    znacznikCzasu=0;
                    pomiar=0;
                    zliczenie++;
                    Serial.print("\t");
                    Serial.print("Zliczenie nr: ");
                    Serial.print(zliczenie);
                    mb.Ireg(SENSOR_IREG, zliczenie);
                }
            }

            if(znacznikCzasu==1 && i>(srednia+amp*srednia)){
                stoperStart=millis();
                pomiar=1;
            }

            if(pomiar == 0){
                tab[cod%wielkoscTab]=i;//wpisywanie wartości do tablicy sredniej
                for(int dlugosc = 0; dlugosc<wielkoscTab; dlugosc++){
                    sredniaSuma+=tab[dlugosc];//wyliczana suma do obliczenia sredniej
                }
                srednia=sredniaSuma/wielkoscTab;//wyliczanie sredniej
            }

            if(i<srednia+0.7*srednia){
                if(pomiar==1){
                    stoperStop=millis()-Tpomiaru;//czas trwania pomiaru
                }else{
                    czasZegara=0;
                    zegar=0;
                }
            }

            sredniaSuma=0;//zerwonie sumy sredniej
            cod++; //licznik wykonanych pętli
            Serial.println();
        }
        break;
        //KONIEC WERSJI 1
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        case 1:
        Serial.println("Konfiguracja Odbiciowa CZESTOTLIWOSCIOWA");

        //WERSJA 2 - ANALIZA CZESTOTLIWOSCIOWA
        //Uruchamianie pętli programu
        sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
        while(true){
            mb.task();

            /*SAMPLING*/
            for(int i=0; i<SAMPLES; i++)
            {
                microseconds = micros();    //Overflows after around 70 minutes!

                vReal[i] = analogRead(0);
                vImag[i] = 0;

                while(micros() < (microseconds + sampling_period_us)){
                }
            }

            /*FFT*/
            FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
            FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
            double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

            /*PRINT RESULTS*/
            Serial.print(peak); Serial.print("\t");Serial.print(seria); Serial.print("\t"); Serial.print(znacznikCzasu*100); Serial.print("\t"); Serial.println(zliczenie);     //Print out what frequency is the most dominant.
            if(peak > 450){
                pomiar=1;
                seria++;
                if(seria > 2){
                    znacznikCzasu=1;
                }
            }else{
                if(znacznikCzasu==1){
                    zliczenie++;
                    znacznikCzasu = 0;
                    mb.Ireg(SENSOR_IREG, zliczenie);
                }
                pomiar = 0;
                seria = 0;
            }
        }
        //KONIEC WERSJI 2
        break;
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 2:
        Serial.println("Konfiguracja Transmisyjna Amplitudowa");

        while(true){
            mb.task();
            i = analogRead(OdczytFotodiody);
            Serial.print(i);

            if(cod > wielkoscTab && i<srednia && zegar==0){//prawda gdy pętla się skalibruje
                stoperStart=millis();
                zegar=1;
            }

            if(zegar==1){
                czasZegara= millis()-stoperStart;
                if(czasZegara > okres*2) pomiar=1;
            }

            if(pomiar == 0){
                tab[cod%wielkoscTab]=i;//wpisywanie wartości do tablicy sredniej
                for(int dlugosc = 0; dlugosc<wielkoscTab; dlugosc++){
                    sredniaSuma+=tab[dlugosc];//wyliczana suma do obliczenia sredniej
                }
                srednia=sredniaSuma/wielkoscTab;//wyliczanie sredniej
            }

            Serial.print("\t");
            Serial.print(srednia);
            Serial.print("\t");
            Serial.print(pomiar*100);

            if(i>srednia){
                if(pomiar==1){
                    zliczenie++;
                    stoperStop=millis()-stoperStart;//czas trwania pomiaru
                    Serial.print("\t");
                    Serial.print("Zliczenie nr: ");
                    Serial.println(zliczenie);
                    mb.Ireg(SENSOR_IREG, zliczenie);
                }
                pomiar=0 ;
                czasZegara=0;
                zegar=0;
            }

            sredniaSuma=0;//zerwonie sumy sredniej
            cod++; //licznik wykonanych pętli
            Serial.println();
        }
        break;
    }
}
