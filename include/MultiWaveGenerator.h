/**
 * @file MultiWaveGenerator.h
 * @author Phil Schatzmann
 * @brief A simple sine wave generator 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <stdint.h>

typedef enum{
	WAV_SINE	= 0x0000,
	WAV_PWM	= 0x0001,
	WAV_SAW	= 0x0002,
    WAV_FARFISA =0x0003,
    WAV_MOOG = 0x0004,
}waveType;

class MultiWaveGenerator {

    public:

        // the scale defines the max value which is generated
        MultiWaveGenerator(float amplitude = 32767.0, float phase = 0.0, waveType wt = WAV_SINE, float pwm = PI,int invert = 0){
            this->m_amplitude = 32767;
            this->m_waveType = wt;
            this->m_PWM = pwm;
            this->m_phase = phase; // this is the starting phase of the wave
            this->m_invert = invert; // invert the wave about amplitude/2
        
        }                                                                                                                 

        /// Defines the frequency - after the processing has been started
        void setFrequency(uint16_t frequency) {
            this->m_frequency = frequency;
        }

        void setPhase(float ph){
            this->m_phase=ph;
        }

        void setAmplitude(float amp){
            this->m_amplitude=amp;
        }

        void setAmplitude01(float amp){
            this->m_amplitude=32767.0*amp;
        }

        void setSampleRate(uint16_t sr){
            sample_rate = sr;
            this->m_deltaTime = 1.0 / sample_rate;
        }
        
        void setWaveType(waveType wt){
            this->m_waveType = wt;
        }
        /// Provides a single sample
        int16_t readSample() { /// this is not used
            float angle = double_Pi * m_frequency * m_time + m_phase;
            int16_t result = m_amplitude * sin(angle);
            if (angle>double_Pi)
            {
                m_time =(angle-double_Pi)/m_frequency;
            }
            m_time += m_deltaTime;
            return result;
        }

        int16_t readSample2() {// this is the code we use
            // m_phase is the starting phase of the waveform
            // g_phase is the advancing phase that increments each time
            int16_t result;
            float ramp;
            switch (m_waveType)
            {
            case WAV_SINE:
                // how long does this sine calculation take??
                result = m_amplitude * sin(g_phase+m_phase);
                break;
            
            case WAV_PWM:
                result =-m_amplitude;
                if(g_phase+m_phase>m_PWM){
                    result = m_amplitude;
                }
                break;
            case WAV_SAW:
                //rising slope saw, centered around zero
                //PWM <2Pi turns it into a pulsed saw like a moog
                ramp=(m_PWM-(g_phase+m_phase))/m_PWM; // fraction of saw portion starts at one and reaches zero at completon
                if(ramp>0){// the rising edge
                    result = (-m_amplitude)+(1-ramp)*m_amplitude;
                }
                //else{
                //    result=-m_amplitude;
                //}
                break;         
           default:
                result = m_amplitude * sin(g_phase+m_phase);
                break;
            }
            

            // invert the wave around zero if necessary
            if(m_invert!=0){
                //result = (m_amplitude/2)-(result-(m_amplitude/2));
                //result = m_amplitude-result;
                result = m_amplitude-result;
            }
            
            
            // increment the phase angle by one sample time
            g_phase += double_Pi * m_frequency * m_deltaTime;
            if(g_phase+m_phase > double_Pi)
            {
			    g_phase -= double_Pi;
            }
            return result;
        }



        /// filles the data with 2 channels
        size_t read(uint8_t *buffer, size_t bytes){
            size_t result = 0;
            int16_t *ptr = (int16_t*)buffer;
            for (int j=0;j<bytes/4;j++){
                int16_t sample = readSample2();
                *ptr++ = sample;
                *ptr++ = sample;
                result+=4;
            }
            return result;
        }

    protected:
        int sample_rate;
        int m_waveType = WAV_SINE;
        float m_PWM=PI;
        int m_invert =0;
        float m_frequency = 0;
        float m_time = 0.0;
        float m_amplitude = 32767.0;  
        float m_deltaTime = 0.0;
        float m_phase = 0.0;
        float double_Pi = PI * 2.0;
        float g_phase =0;

};