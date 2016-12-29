#ifndef JPDimmerLib_H
#define JPDimmerLib_H

#include <Arduino.h>

#define JPDimmer_Create(INTERRUPT_PIN, SYNC_CALL, AMOUNT_OF_LAMPS, STEPS_PER_CYCLE) \
        JPDimmerLib JPDimmer(STEPS_PER_CYCLE, AMOUNT_OF_LAMPS, INTERRUPT_PIN, SYNC_CALL); \
        ISR(TIMER1_COMPA_vect) { \
                JPDimmer.update(); \
        } \
        volatile uint32_t JPDimmerLib::zerocrossinginterrupttime = 0;

class JPDimmerLib
{
public:

        // preferably use JPDimmer_Create
        JPDimmerLib(uint8_t stepspercycle, uint8_t amountoflamps, int zerocrossing_interruptpin, void (*synccall_)(const boolean),
                    uint8_t minimumbrightness = 19, uint16_t zerocrossingoffset = 1800,
                    uint8_t stepsaftertimerdisable = 10, uint8_t stepsneededforsync = 20,
                    double tu_ = 1, double ku_ = 0.06, float gamma = 1);

        void lampinit(uint8_t lampnumber, int pin);

        void fadeto(uint8_t lampnumber, uint8_t brightness, uint16_t steps, unsigned long duration, unsigned long startdelay);
        void fadetolast(uint8_t lampnumber, uint16_t steps, unsigned long duration, unsigned long startdelay);
        void setbrightness(uint8_t lampnumber, uint8_t brightness);
        void setlastbrightness(uint8_t lampnumber);
        uint8_t getrequestedbrightness(uint8_t lampnumber);
        uint8_t getactualbrightness(uint8_t lampnumber);
        boolean getfading(uint8_t lampnumber);
        boolean getinited(uint8_t lampnumber);
        boolean getactivity(); // of all lamps
        boolean getsyncstatus();

        void loop();

        // used internally only but needed to make public:

        void update() __attribute__((always_inline))
        {
                if (phase == STEPS_PER_CYCLE - 1 - ZEROCROSSING_CYCLE_OFFSET)
                {
                        phaseinterrupttime = micros();
                }
                OCR1A = OCR1Anew;
                if (insync)
                {
                        PORTB = (portBprogram[phase]) | (portBresetmask & PORTB);
                        PORTC = (portCprogram[phase]) | (portCresetmask & PORTC);
                        PORTD = (portDprogram[phase]) | (portDresetmask & PORTD);
                }
                if (!phase)
                {
                        phase = STEPS_PER_CYCLE - 1;
                }
                else
                {
                        phase--; // prepare for next phase;
                }
        }

        static volatile uint32_t zerocrossinginterrupttime;

private:

        void phasecorrection();
        void ACperiodicstuff();
        boolean requestbrightness(uint8_t lampnumber, uint8_t brightness); // this can fail (returns false then)!!
        void fadeLoop(uint8_t currentlamp);
        void setuptimer1();
        void starttimer1();
        void stoptimer1();
        void disabletriacs();

        const uint8_t STEPS_PER_CYCLE;
        const uint8_t AMOUNT_OF_LAMPS;
        const uint8_t ZEROCROSSING_CYCLE_OFFSET;
        const uint16_t OCR1A_DEFAULT;
        const uint16_t ZEROCROSSING_OFFSET;
        const uint8_t STEPS_AFTER_TIMER_DISABLE; // after how many AC periods to disablt the timer
        const uint8_t STEPS_NEEDED_FOR_SYNC; // how many AC periods needed in sync to flag in sync

        uint8_t portBresetmaskinit;
        uint8_t portCresetmaskinit;
        uint8_t portDresetmaskinit;

        volatile uint8_t *portBprogram;
        volatile uint8_t portBresetmask;
        volatile uint8_t *portCprogram;
        volatile uint8_t portCresetmask;
        volatile uint8_t *portDprogram;
        volatile uint8_t portDresetmask;

        uint32_t zerocrossinginterrupttime_d;
        volatile uint32_t phaseinterrupttime;
        uint32_t phaseinterrupttime_d;

        volatile int16_t OCR1Anew;

        volatile uint8_t phase;

        const double tu;
        const double ku;

        const double kp;
        const double ki;
        const double kd;

        uint32_t burninglampoverview;
        uint32_t fadingoverview;

        int32_t timedifference, timedifference_d;
        double integrator, error, derivative;
        const int32_t margin;
        uint8_t crossed;
        uint8_t synclevel;
        boolean insync;
        void (*synccall)(const boolean);

//        const uint8_t minimum_brightness;

        typedef struct
        {
                volatile uint8_t *addresstoprogram;
                volatile uint8_t *addresstoreset;
                uint8_t bitinprogram;
                uint8_t setting;
                uint8_t brightness;
                uint8_t lastbrightness;

                //fading stuff
                uint32_t fadestarttime;
                uint32_t fadedelay;
                uint32_t fadelastsyncchecktime;
                boolean fading;
                uint16_t fadesteps, lastfadestep;
                uint8_t requested_brightness;
                uint8_t start_brightness;
                int32_t step;
                uint8_t addresstoclear;
        } lamp;

        uint8_t gammatable[101]; // input brightness is 0..100 so 101 entries for gamma

        lamp *lamps;

        static void zeroCrossingInterrupt()  __attribute__((always_inline))
        {
                JPDimmerLib::zerocrossinginterrupttime = micros();
        }
};
#endif
