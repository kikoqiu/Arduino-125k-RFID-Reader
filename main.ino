#define DEBUG 0
 
#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif
 
 
#include <avr/io.h>
#include "wiring_private.h"
#include "pins_arduino.h"
 
#define ADCPIN 0
void initADC(void);
void Reader();
void Decode();
 
const int data_length=1024;
//the buffer for ADC data
unsigned char buf[data_length];
 
 
 
 
void setup()
{
        Serial.begin(115200);
        initADC();
        pinMode(9,OUTPUT);
}
 
void loop(){
        Reader();
}
 
 
 
 
 
 
//The Reader function OUTPUT the 125k signal from d9 port and caputures the result signal via A0 port. 
//Then it decode the result.
//author kikoqiu[at]163.com
void Reader()
{        
        u16 bufpos=0;
        //we sample for every 16 loops.
        char sampleTimes=0;
 
        //the loop executed for every 64 cpu cycles
        while(bufpos<data_length){                
                //invert the port 9 digital output pin, that makes the 125k hz wave.
                PORTB=PORTB^2;
                //pad cycles up
                __asm__ __volatile__ (
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        );
 
                int s=(sampleTimes%16);
                if(s==0){
                        //we start adc conversion
                        sbi(ADCSRA, ADSC);
                        bufpos++;        
                }else{
                        //we save the adc result ( it will be executed for 15 times, and we only need the last one)
                        //but we keep it like this because we need the loop to be executed every 1/125k/2 seconds, 
                        //so we keep as little difference as possible for every loop, i.e., we have to
                        //execute for 64 cpu cycles in every loop.
                        buf[bufpos]=ADCH;
                        //the two nop is used to pad the cpu cycles up with the 'if' brunches.
                        __asm__ __volatile__ (
                                "NOP\n\t"
                                "NOP\n\t");
                }
                sampleTimes++;
 
                //pad cycles up
                __asm__ __volatile__ (
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        "NOP\n\t"
                        );
 
        }
 
        //debug
        /*for(int i=0;i<bufpos;++i){
                Serial.print((int)buf[i]);
                Serial.print(",");
        }*/
 
        Decode();
        bufpos=0;
}
 
void Decode(){
        char decoded[100];
        int codepos=0;
 
        //the start up bits of manchester coded value for start signal(0,1,1,1,1,1,1,1,1,1)
        char startBits[]={0,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};
        int startBitPos=0;
 
        //previous signal
        char prevsig=0;
        //previous edge position
        int prevPos=0;
 
        //the signal length( As we sampleTimes once for every 16 signals, we got 64/16=4 samples per changement
        const int averageLen=4;
 
        //the averageSignalStrength value of signal
        u32 averageSignalStrength=0;
        for(int i=0;i<64;++i){        
                averageSignalStrength+=(u32)buf[i]*1000/64;
        }
        averageSignalStrength/=1000;
        dprint(averageSignalStrength);
 
 
        //every two signal result in one bit in manchester encoding, so we mark for every two signal
        char signalCount=0;
        for(int i=1;i<data_length-1;++i){                
                char sig=buf[i]<(u8)averageSignalStrength?1:0;
                char nextsig=buf[i+1]<(u8)averageSignalStrength?1:0;
                //do some basic filtering (if [0,1,0],then [0,0,0]. if [1,0,1],then [1,1,1].)
                if(sig==0&&prevsig==1&&nextsig==1){
                        sig=1;
                }
                if(sig==1&&prevsig==0&&nextsig==0){
                        sig=0;
                }
 
                //if on the edge
                if(sig^prevsig){
                        int len=i-prevPos;
                        int time=1;
                        //if signal length> threshole, we count it as two.)
                        if(len > averageLen*3/2){
                                time=2;
                        }
                         
                        while(time-->0){
                                //match for the start bits
                                if(startBitPos!=sizeof(startBits)){
                                        if(startBits[startBitPos]==sig){
                                                ++startBitPos;
                                        }else{
                                                startBitPos=0;
                                        }
                                }else{                                
                                        //it's matched (already), we start decode here
                                        if(signalCount==0){//go for the next signal
                                                signalCount=1;
                                        }else{
                                                signalCount=0;
                                                if(sig){//0,1 = 0
                                                        decoded[codepos++]=0;
                                                        if(codepos==55){
                                                                break;
                                                        }
                                                }else{//1,0 = 1
                                                        decoded[codepos++]=1;
                                                        if(codepos==55){
                                                                break;
                                                        }
                                                }
                                        }
                                }                                        
                        }
                        //remember the previous position
                        prevPos=i;
                }        
                //remember the previous signal
                prevsig=sig;
                //we get enough code (64 all - 9 start bits =55)
                if(codepos==55){
                        break;
                }
        }
 
         
        bool checkfailed=false;
        char result[40];
        int resultPos=0;
        dprint(codepos);
 
        /*for(int i=0;i<55;++i){
                Serial.print((int)decoded[i]);
                Serial.print(",");
        }
        Serial.println();*/
        if(codepos==55){                
                for(int i=0;i<10;++i){
                        char r=decoded[i*5+4];
                        for(int j=0;j<4;++j){
                                r^=decoded[i*5+j];        
                                result[resultPos++]=decoded[i*5+j];
                        }
                        if(r!=0){
                                checkfailed=true;
                                break;
                        }
                }
                for(int i=0;i<4;++i){
                        char r=0;
                        for(int j=0;j<11;++j){
                                r^=decoded[j*5+i];
                        }
                        if(r!=0){
                                checkfailed=true;
                                break;
                        }
                }
 
                if(!checkfailed){
                        unsigned re=0,r0=0;
                        for(int i=8;i<40;++i){
                                re<<=1;
                                re^=result[i];
                        }
                        for(int i=0;i<8;++i){
                                r0<<=1;
                                r0^=result[i];
                        }
                        Serial.println(r0);
                        Serial.println(re);
                }
        }
}
 
 
 
/*Init ADC
Reference = AVCC with external capacitor at AREF pin
ADC clock = main clock / 16
Manual ADC read
No ADC interrupt
Disable digital buffer of ADC pin 
*/
void initADC(void)
{
        //---------------------------------------------------------------------
        // ADMUX settings
        //---------------------------------------------------------------------
        // These bits select the voltage reference for the ADC. If these bits
        // are changed during a conversion, the change will not go in effect
        // until this conversion is complete (ADIF in ADCSRA is set). The
        // internal voltage reference options may not be used if an external
        // reference voltage is being applied to the AREF pin.
        // REFS1 REFS0 Voltage reference
        // 0 0 AREF, Internal Vref turned off
        // 0 1 AVCC with external capacitor at AREF pin
        // 1 0 Reserved
        // 1 1 Internal 1.1V Voltage Reference with external
        //   capacitor at AREF pin
        cbi(ADMUX,REFS1);
        sbi(ADMUX,REFS0);
        // The ADLAR bit affects the presentation of the ADC conversion result
        // in the ADC Data Register. Write one to ADLAR to left adjust the
        // result. Otherwise, the result is right adjusted. Changing the ADLAR
        // bit will affect the ADC Data Register immediately, regardless of any ongoing conversions.
        sbi(ADMUX,ADLAR);
        // The value of these bits selects which analog inputs are connected to
        // the ADC. If these bits are changed during a conversion, the change
        // will not go in effect until this conversion is complete (ADIF in ADCSRA is set).
        ADMUX |= ( ADCPIN & 0x07 );
 
        //---------------------------------------------------------------------
        // ADCSRA settings
        //---------------------------------------------------------------------
        // Writing this bit to one enables the ADC. By writing it to zero, the
        // ADC is turned off. Turning the ADC off while a conversion is in
        // progress, will terminate this conversion.
        if(false)cbi(ADCSRA,ADEN);
        // In Single Conversion mode, write this bit to one to start each
        // conversion. In Free Running mode, write this bit to one to start the
        // first conversion. The first conversion after ADSC has been written
        // after the ADC has been enabled, or if ADSC is written at the same
        // time as the ADC is enabled, will take 25 ADC clock cycles instead of
        // the normal 13. This first conversion performs initialization of the
        // ADC. ADSC will read as one as long as a conversion is in progress.
        // When the conversion is complete, it returns to zero. Writing zero to
        // this bit has no effect.
        cbi(ADCSRA,ADSC);
        // When this bit is written to one, Auto Triggering of the ADC is
        // enabled. The ADC will start a conversion on a positive edge of the
        // selected trigger signal. The trigger source is selected by setting
        // the ADC Trigger Select bits, ADTS in ADCSRB.
        cbi(ADCSRA,ADATE);
        // When this bit is written to one and the I-bit in SREG is set, the
        // ADC Conversion Complete Interrupt is activated.
        if(false)sbi(ADCSRA,ADIE);
        // These bits determine the division factor between the system clock
        // frequency and the input clock to the ADC.
        // ADPS2 ADPS1 ADPS0 Division Factor
        // 0 0 0 2
        // 0 0 1 2
        // 0 1 0 4
        // 0 1 1 8
        // 1 0 0 16
        // 1 0 1 32
        // 1 1 0 64
        // 1 1 1 128
        sbi(ADCSRA,ADPS2);
        cbi(ADCSRA,ADPS1);
        cbi(ADCSRA,ADPS0);
 
        //---------------------------------------------------------------------
        // ADCSRB settings
        //---------------------------------------------------------------------
        // When this bit is written logic one and the ADC is switched off
        // (ADEN in ADCSRA is zero), the ADC multiplexer selects the negative
        // input to the Analog Comparator. When this bit is written logic zero,
        // AIN1 is applied to the negative input of the Analog Comparator.
        //cbi(ADCSRB,ACME);
        // If ADATE in ADCSRA is written to one, the value of these bits
        // selects which source will trigger an ADC conversion. If ADATE is
        // cleared, the ADTS2:0 settings will have no effect. A conversion will
        // be triggered by the rising edge of the selected Interrupt Flag. Note
        // that switching from a trigger source that is cleared to a trigger
        // source that is set, will generate a positive edge on the trigger
        // signal. If ADEN in ADCSRA is set, this will start a conversion.
        // Switching to Free Running mode (ADTS[2:0]=0) will not cause a
        // trigger event, even if the ADC Interrupt Flag is set.
        // ADTS2 ADTS1 ADTS0 Trigger source
        // 0 0 0 Free Running mode
        // 0 0 1 Analog Comparator
        // 0 1 0 External Interrupt Request 0
        // 0 1 1 Timer/Counter0 Compare Match A
        // 1 0 0 Timer/Counter0 Overflow
        // 1 0 1 Timer/Counter1 Compare Match B
        // 1 1 0 Timer/Counter1 Overflow
        // 1 1 1 Timer/Counter1 Capture Event
        cbi(ADCSRB,ADTS2);
        cbi(ADCSRB,ADTS1);
        cbi(ADCSRB,ADTS0);
 
        //---------------------------------------------------------------------
        // DIDR0 settings
        //---------------------------------------------------------------------
        // When this bit is written logic one, the digital input buffer on the
        // corresponding ADC pin is disabled. The corresponding PIN Register
        // bit will always read as zero when this bit is set. When an analog
        // signal is applied to the ADC5..0 pin and the digital input from this
        // pin is not needed, this bit should be written logic one to reduce
        // power consumption in the digital input buffer.
        // Note that ADC pins ADC7 and ADC6 do not have digital input buffers,
        // and therefore do not require Digital Input Disable bits.
        sbi(DIDR0,ADC5D);
        sbi(DIDR0,ADC4D);
        sbi(DIDR0,ADC3D);
        sbi(DIDR0,ADC2D);
        sbi(DIDR0,ADC1D);
        sbi(DIDR0,ADC0D);
}
