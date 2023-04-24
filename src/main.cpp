#include <Arduino.h>
#include "AudioKitHAL.h"
//#include "SineWaveGenerator.h"
#include "MultiWaveGenerator.h"


AudioKit kit;
//SineWaveGenerator wave;
//MultiWaveGenerator wave; // uses default wave settings
// MultiWaveGenerator(float amplitude = 32767.0, float phase = 0.0, waveType wt = WAV_SINE, float pwm = PI,int invert = 0)
MultiWaveGenerator CAR_wave(32767.0,0.0,WAV_SINE,2*PI,0);
MultiWaveGenerator FM_wave(32767.0,0.0,WAV_SINE,2*PI,0);
MultiWaveGenerator AM_wave(32767.0,0.0,WAV_SINE,2*PI,0);
const int BUFFER_SIZE = 1024; // this is the buffer size in bytes, not samples
uint8_t buffer[BUFFER_SIZE];
uint8_t MTbuffer[BUFFER_SIZE];

const int DELAY_BUFFER_SIZE = 24350*4; // one second of delay with four bytes per sample (really don't need stereo)
uint8_t delay_buffer[DELAY_BUFFER_SIZE];
float maxamp =32767.0;
int DELAY = 1020;
uint16_t FM_WF=0;
float FM_FQ =6;
float FM_AMT =0;
uint16_t CAR_FQ =3000;
uint16_t AM_WF =0;
float AM_FQ =0;
float AM_AMT =0;
uint16_t CAR_VOL =0;
uint16_t DEL_RT =23000;
float DEL_AMT=0;
float DEL_FB=0;
int16_t FM_instant;
int16_t AM_instant;
float AM_float;
uint16_t CAR_FQ_READ;
uint16_t CAR_FQ_READ_NEW;


size_t bytes2i2s;
int16_t* del_ptr_write;
int16_t* del_ptr_read;
int16_t* del_ptr_start;
int16_t* del_ptr_end;

const int firePin = 12;  // the number of the pushbutton pin should re 'rec' button on LyraT
int fireState = 1;  // input goes low when fired

const int FM_WF_PIN = 13;
const int FM_FQ_PIN = 39;
const int FM_AMT_PIN = 34;
const int CAR_FQ_PIN = 32;
const int AM_WF_PIN = 33;
const int AM_FQ_PIN = 27;
const int AM_AMT_PIN = 14;
const int CAR_VOL_PIN = 15;
const int DEL_RT_PIN = 36;
const int DEL_AMT_PIN = 4;
const int DEL_FB_PIN = 2;

int freqPotValue=1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  log_i("before malloc free heap, %1fkb",(float)esp_get_free_heap_size()/1024);
  LOGLEVEL_AUDIOKIT = AudioKitInfo; 
  MTbuffer[BUFFER_SIZE] = {0};
  delay_buffer[DELAY_BUFFER_SIZE]={0};
  void *pttr = malloc (50*1024);
  log_i("after malloc free heap, %1fkb",(float)esp_get_free_heap_size()/1024);
  //LOGLEVEL_AUDIOKIT = AudioKitDebug;  
  
  //Serial.print("begin");
  // open in write mode
  auto cfg = kit.defaultConfig(AudioOutput);
  kit.begin(cfg);
  kit.setVolume(30);

  del_ptr_write = (int16_t*)delay_buffer;
  del_ptr_start = (int16_t*)delay_buffer;
  //del_ptr_end = (int16_t*)delay_buffer[DELAY_BUFFER_SIZE-4]; // should point to the last element
  del_ptr_end = del_ptr_start+(DELAY_BUFFER_SIZE/4);
  //del_ptr_read = (int16_t*)delay_buffer[DELAY_BUFFER_SIZE-10000]; // sets delay to read head
  del_ptr_read = del_ptr_end - (23000*4);
  Serial.print("del_ptr_start/n");
  Serial.print((int)del_ptr_start);
  Serial.print("del_ptr_end/n");
  Serial.print((int)del_ptr_end);
  Serial.print("del_ptr_read/n");
  Serial.print((int)del_ptr_read);
    Serial.print("del_ptr_write/n");
  Serial.print((int)del_ptr_write);
  pinMode(firePin, INPUT_PULLUP);
  //CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_DP_PULLUP);
  

  // 1000 hz
  CAR_wave.setFrequency(CAR_FQ);
  CAR_wave.setSampleRate(cfg.sampleRate());
  AM_wave.setFrequency(AM_FQ);
  AM_wave.setSampleRate(cfg.sampleRate());
  FM_wave.setFrequency(FM_FQ);
  FM_wave.setSampleRate(cfg.sampleRate());
  //size_t bufferfilledsize = wave.read(buffer, BUFFER_SIZE);
}

void loop() {
  // put your main code here, to run repeatedly:

  FM_AMT = 1-((float)analogRead(FM_AMT_PIN)/4096);
  AM_AMT = 1-((float)analogRead(AM_AMT_PIN)/4096);
  AM_FQ = 20*(((float)(4096-analogRead(AM_FQ_PIN)))/4096);
  AM_wave.setFrequency(AM_FQ);
  FM_FQ = 20*(((float)(4096-analogRead(FM_FQ_PIN)))/4096);
  FM_wave.setFrequency(FM_FQ);
  CAR_FQ_READ_NEW = analogRead(CAR_FQ_PIN);
  if (abs(CAR_FQ_READ_NEW-CAR_FQ_READ)>150){
    CAR_FQ_READ=CAR_FQ_READ_NEW;
  }
  //FM_instant = FM_AMT*100*(float)(FM_wave.readSample2()/ 32767.0);
  //FM_instant = FM_wave.readSample2();
  //AM_instant = AM_wave.readSample2();
  //CAR_FQ = (FM_instant)+100+(4095- analogRead(CAR_FQ_PIN));
  //CAR_FQ = (10*(FM_AMT*FM_instant/ 32767.0))+100+(4095- analogRead(CAR_FQ_PIN));
  //CAR_wave.setFrequency(CAR_FQ);
  size_t bytes =  BUFFER_SIZE;
  size_t result = 0;
  int16_t *ptr = (int16_t*)buffer;
  fireState = digitalRead(firePin); 
  for (int j=0;j<bytes/4;j++){
        FM_instant = FM_wave.readSample2();
        AM_instant = AM_wave.readSample2();
        //AM_float = (1-(AM_AMT*((float)AM_instant/32767)));
        AM_float = (AM_AMT)*((float)AM_instant/32767); // -1 to 1
        AM_float = 0.5f * AM_float; // 0.5 to -0.5
        AM_float = AM_float + 0.5; // 0 to 1
        AM_float = 1.5 - (AM_AMT/2) - AM_float;
        CAR_FQ = (200*(FM_AMT*FM_instant/ 32767.0))+100+(4095- CAR_FQ_READ);
        CAR_wave.setFrequency(CAR_FQ);
       //int16_t sample =  (1-(AM_AMT*AM_wave.readSample2()/ 32767.0)) * CAR_wave.readSample2();
       //int16_t sample =   (int)((1-(AM_AMT*AM_instant/ 32767.0)) * (float)CAR_wave.readSample2());
       int16_t sample =   (int16_t)(AM_float * (float)CAR_wave.readSample2());
       if (fireState==HIGH)
       {sample=0;}
       int16_t delay_sample = *del_ptr_read++;
       delay_sample = *del_ptr_read++;
       //int16_t mix = (sample/2) + (delay_sample/2);
       int16_t mix = sample;
      
      *ptr++ = mix;
      *ptr++ = mix;
      *del_ptr_write++ = (sample/2) + (delay_sample/2);
      *del_ptr_write++ = (sample/2) + (delay_sample/2);

      //result+=4;
      if (del_ptr_read>=del_ptr_end){
        del_ptr_read=del_ptr_start; // del_ptr_read overflow
        }
      
        if (del_ptr_write>=del_ptr_end){
          del_ptr_write=del_ptr_start;
        }

      


  }



  bytes2i2s=kit.write(buffer, 1024);
  if (fireState == HIGH) {
    // fire button is NOT pressed, no sound
     //bytes2i2s=kit.write(MTbuffer, 1024);
  } else {
  //   Serial.print("PUshez\n");
  //   Serial.print("CAR_FQ\n");
  // Serial.print((int)CAR_FQ);
  //     Serial.print("\nFM_FQ\n");
   //Serial.print(FM_FQ);
         Serial.print("\nAM_float\n");
   Serial.print(AM_float);
  //         Serial.print("\nFM_instant\n");
  // Serial.print(FM_instant);
  //       Serial.print("\nAM_FQ\n");
  // Serial.print(AM_FQ);
  //           Serial.print("\nAM_instant\n");
  // Serial.print(AM_instant);

    // fire button is  pressed, sound on
    // bytes2i2s=kit.write(buffer, 1024);
  }


 
  //Serial.print(bytes2i2s);
  if (bytes2i2s<BUFFER_SIZE)
  {
    Serial.print("OVA_FLO1");
  }



  DELAY = DELAY-1;
  if (DELAY < 1)
  {
    // print this somehow for debug float i2s_get_clk(i2s_port_t i2s_num);
    // int DELAY = 10;
    // carrier_freq= carrier_freq +100;
    // carrier_wave.setFrequency(carrier_freq);
    // if (carrier_freq > 5000){
    //   carrier_freq =300;
    // }


  }
}