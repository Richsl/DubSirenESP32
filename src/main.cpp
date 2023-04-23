#include <Arduino.h>
#include "AudioKitHAL.h"
//#include "SineWaveGenerator.h"
#include "MultiWaveGenerator.h"


AudioKit kit;
//SineWaveGenerator wave;
//MultiWaveGenerator wave; // uses default wave settings
// MultiWaveGenerator(float amplitude = 32767.0, float phase = 0.0, waveType wt = WAV_SINE, float pwm = PI,int invert = 0)
MultiWaveGenerator carrier_wave(32767.0,0.0,WAV_SINE,2*PI,0);
MultiWaveGenerator mod_wave(32767.0,0.0,WAV_PWM,PI,0);
const int BUFFER_SIZE = 1024; // this is the buffer size in bytes, not samples
uint8_t buffer[BUFFER_SIZE];
uint8_t MTbuffer[BUFFER_SIZE];

const int DELAY_BUFFER_SIZE = 24350*4; // one second of delay with four bytes per sample (really don't need stereo)
uint8_t delay_buffer[DELAY_BUFFER_SIZE];
float maxamp =32767.0;
int DELAY = 1020;
uint16_t carrier_freq =3000;
uint16_t mod_freq =6;
size_t bytes2i2s;
int16_t* del_ptr_write;
int16_t* del_ptr_read;
int16_t* del_ptr_start;
int16_t* del_ptr_end;

const int firePin = 36;  // the number of the pushbutton pin should re 'rec' button on LyraT
int fireState = 1;  // input goes low when fired
const int carrierFreqPotPin = 15;
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
  kit.setVolume(40);

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
  pinMode(firePin, INPUT);
  

  // 1000 hz
  carrier_wave.setFrequency(carrier_freq);
  carrier_wave.setSampleRate(cfg.sampleRate());
  mod_wave.setFrequency(mod_freq);
  mod_wave.setSampleRate(cfg.sampleRate());
  //size_t bufferfilledsize = wave.read(buffer, BUFFER_SIZE);
}

void loop() {
  // put your main code here, to run repeatedly:
  freqPotValue = analogRead(carrierFreqPotPin);
  carrier_wave.setFrequency(freqPotValue+100);
  size_t bytes =  BUFFER_SIZE;
  size_t result = 0;
  int16_t *ptr = (int16_t*)buffer;
  fireState = digitalRead(firePin); 
  for (int j=0;j<bytes/4;j++){
       int16_t sample =  (mod_wave.readSample2()/ 32767.0) * carrier_wave.readSample2();
       if (fireState==HIGH)
       {sample=0;}
       int16_t delay_sample = *del_ptr_read++;
       delay_sample = *del_ptr_read++;
       int16_t mix = (sample/2) + (delay_sample/2);
       //int16_t mix = sample;
      
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
    int DELAY = 10;
    carrier_freq= carrier_freq +100;
    carrier_wave.setFrequency(carrier_freq);
    if (carrier_freq > 5000){
      carrier_freq =300;
    }


  }
}