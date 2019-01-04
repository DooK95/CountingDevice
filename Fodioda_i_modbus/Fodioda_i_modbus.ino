#include <EtherCard.h>
#include <Modbus.h>
#include <ModbusIP_ENC28J60.h>
#include <arduinoFFT.h>
#include <EEPROM.h>

#define SAMPLES 32         //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
#define INPUT_PIN A0
#define TAB_SIZE 400

arduinoFFT FFT = arduinoFFT();

unsigned int i,loopCounter=0, tab[TAB_SIZE],average, max, min, period, frequency=500/*f lasera w Hz*/, sampling_period_us;
bool isDetected=0, isClockCounting=0, isLongerThanPeriods=0;
int mesurmentSeries=0, stopwatchTime, configuration = 1; //0 - Odbiciowa Amplitudowa, 1 - Odbiciowa frequencyiowa, 2 - Transmisyjna Amplitudowa
unsigned long averageSum, stopwatchStart, detectionTime, counter=0, microseconds;
float amp=0.3;//wzmocnienie progu aktywacji
double vReal[SAMPLES], vImag[SAMPLES];

//Modbus Registers Offsets (0-9999)
const int SENSOR_IREG = 100;


void setup() {

    //---Modbus Declaration---
    //ModbusIP object
    ModbusIP mb;
    //The media access control (ethernet hardware) address for the shield
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
    //The IP address for the shield
    byte ip[] = { 192, 168, 0, 24 };
    //Config Modbus IP
    mb.config(mac, ip);
    //Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addIreg(SENSOR_IREG);
    //Modbus start
    mb.task();
    //---Modbus Declaration---

    //Ustawiam port szeregowy
    Serial.begin(2000000);

    //Ustawiam przerwania
    attachInterrupt(digitalPinToInterrupt(2), riseF, RISING);
    attachInterrupt(digitalPinToInterrupt(3), changeConf, RISING);

    //Parametry początkowe pracy
    Serial.println("");
    Serial.println("Częstotliwość lasera: "+frequency);
    Serial.println("Zaczynam");

    //Program zaczyna pracę w jednej z konfiguracji //0 - Odbiciowa Amplitudowa, 1 - Odbiciowa frequencyiowa, 2 - Transmisyjna
    switch (configuration) {
        case 0:
        Serial.println("Konfiguracja Odbiciowa AMPLITUDOWA");

        //WERSJA 1 - ANALIZA AMPLITUDOWA
        //Uruchamianie pętli programu
        while(true){
            i = analogRead(INPUT_PIN);//Odczyt z wejścia analogowego
            Serial.print(i);

            if(loopCounter > TAB_SIZE && i>(average+amp*average) && isClockCounting==0){//po wypełnieniu tabeli danymi, pierwszy warunak na rozpoznanie isDetectedu = wykrycie wartości większej
                stopwatchStart=millis();
                isClockCounting=1;
            }

            Serial.print("\t");
            Serial.print((average+amp*average)); //wykreslenie progu aktywacji
            Serial.print("\t");
            Serial.print(isDetected*100);//wyznaczenie czy isDetected trwa na wykresie

            if(isClockCounting==1){
                stopwatchTime= millis()-stopwatchStart;
                if(stopwatchTime <= period*5){
                    isLongerThanPeriods=1;
                }
                else {
                    isLongerThanPeriods=0;
                    isDetected=0;
                    counter++;
                    Serial.print("\t");
                    Serial.print("counter nr: ");
                    Serial.print(counter);
                    mb.Ireg(SENSOR_IREG, counter);
                }
            }

            if(isLongerThanPeriods==1 && i>(average+amp*average)){
                stopwatchStart=millis();
                isDetected=1;
            }

            if(isDetected == 0){
                tab[loopCounter%TAB_SIZE]=i;//wpisywanie wartości do tablicy sredniej
                for(int dlugosc = 0; dlugosc<TAB_SIZE; dlugosc++){
                    averageSum+=tab[dlugosc];//wyliczana suma do obliczenia sredniej
                }
                average=averageSum/TAB_SIZE;//wyliczanie sredniej
            }

            if(i<average+0.7*average){
                if(isDetected==1){
                    detectionTime=millis()-stopwatchStart;//czas trwania isDetectedu
                }else{
                    stopwatchTime=0;
                    isClockCounting=0;
                }
            }

            averageSum=0;//zerwonie sumy sredniej
            loopCounter++; //licznik wykonanych pętli
            Serial.println();
        }
        break;
        //KONIEC WERSJI 1
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        case 1:
        Serial.println("Konfiguracja Odbiciowa Częstotliwościowa");

        //WERSJA 2 - ANALIZA Częstotliwościowa
        //Uruchamianie pętli programu
        sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
        while(true){

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
            Serial.print(peak); Serial.print("\t");Serial.print(mesurmentSeries); Serial.print("\t"); Serial.print(isLongerThanPeriods*100); Serial.print("\t"); Serial.println(counter);     //Print out what frequency is the most dominant.
            if(peak > 450){
                isDetected=1;
                mesurmentSeries++;
                if(mesurmentSeries > 2){
                    isLongerThanPeriods=1;
                }
            }else{
                if(isLongerThanPeriods==1){
                    counter++;
                    isLongerThanPeriods = 0;
                    mb.Ireg(SENSOR_IREG, counter);
                }
                isDetected = 0;
                mesurmentSeries = 0;
            }
        }
        //KONIEC WERSJI 2
        break;
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        case 2:
        Serial.println("Konfiguracja Transmisyjna Amplitudowa");

        while(true){
            i = analogRead(INPUT_PIN);
            Serial.print(i);

            if(loopCounter > TAB_SIZE && i<average && isClockCounting==0){//prawda gdy pętla się skalibruje
                stopwatchStart=millis();
                isClockCounting=1;
            }

            if(isClockCounting==1){
                stopwatchTime= millis()-stopwatchStart;
                if(stopwatchTime > period*2) isDetected=1;
            }

            if(isDetected == 0){
                tab[loopCounter%TAB_SIZE]=i;//wpisywanie wartości do tablicy sredniej
                for(int dlugosc = 0; dlugosc<TAB_SIZE; dlugosc++){
                    averageSum+=tab[dlugosc];//wyliczana suma do obliczenia sredniej
                }
                average=averageSum/TAB_SIZE;//wyliczanie sredniej
            }

            Serial.print("\t");
            Serial.print(average);
            Serial.print("\t");
            Serial.print(isDetected*100);

            if(i>average){
                if(isDetected==1){
                    counter++;
                    detectionTime=millis()-stopwatchStart;//czas trwania isDetectedu
                    Serial.print("\t");
                    Serial.print("counter nr: ");
                    Serial.println(counter);
                    mb.Ireg(SENSOR_IREG, counter);
                }
                isDetected=0 ;
                stopwatchTime=0;
                isClockCounting=0;
            }

            averageSum=0;//zerwonie sumy sredniej
            loopCounter++; //licznik wykonanych pętli
            Serial.println();
        }
        break;
    }
}

void riseF() {
    noInterrupts();
    frequency += 10;
    if (frequency > 500) {
        frequency = 10;
    }
    Serial.println("Częstotliwość lasera: "+frequency+" | 10-500 [Hz]");
    interrupts();
}

void changeConf() {
    noInterrupts();
    configuration += 1;
    if (configuration > 2) {
        configuration = 0;
    }
    Serial.println("Konfiguracja: "+configuration+" | 0 - Odbiciowa Amplitudowa, 1 - Odbiciowa frequencyiowa, 2 - Transmisyjna Amplitudowa");
    interrupts();
}
