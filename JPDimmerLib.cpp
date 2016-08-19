#include "JPDimmerLib.h"

#define AC_PERIOD 10000 // us


JPDimmerLib::JPDimmerLib(uint8_t stepspercycle, uint8_t amountoflamps,
                         int zerocrossing_interruptpin, void (*synccall_)(const boolean),
                         uint8_t minimumbrightness, uint16_t zerocrossingoffset,
                         uint8_t stepsaftertimerdisable, uint8_t stepsneededforsync, double tu_, double ku_) :

        STEPS_PER_CYCLE(stepspercycle),
        AMOUNT_OF_LAMPS(amountoflamps),
        ZEROCROSSING_CYCLE_OFFSET(stepspercycle / 2),
        OCR1A_DEFAULT((F_CPU / (100 * stepspercycle)) - 1),
        ZEROCROSSING_OFFSET(zerocrossingoffset),
        STEPS_AFTER_TIMER_DISABLE(stepsaftertimerdisable),
        STEPS_NEEDED_FOR_SYNC(stepsneededforsync),
        tu(tu_), ku(ku_), kp(ku_ * 0.3), ki(2 * kp / tu_), kd(kp * tu_ / 8),
        minimum_brightness(minimumbrightness)

//minimum_brightness((uint8_t)((zerocrossingoffset * (int32_t)stepspercycle) / AC_PERIOD))

{
        portBresetmaskinit = 0xFF;
        portBresetmask = 0xFF;
        portCresetmaskinit = 0xFF;
        portCresetmask = 0xFF;
        portDresetmaskinit = 0xFF;
        portDresetmask = 0xFF;

        portBprogram = (volatile uint8_t *)malloc(STEPS_PER_CYCLE * sizeof(uint8_t));
        portCprogram = (volatile uint8_t *)malloc(STEPS_PER_CYCLE * sizeof(uint8_t));
        portDprogram = (volatile uint8_t *)malloc(STEPS_PER_CYCLE * sizeof(uint8_t));

        for (int i = 0; i < STEPS_PER_CYCLE; i++)
        {
                portBprogram[i] = 0;
                portCprogram[i] = 0;
                portDprogram[i] = 0;
        }

        zerocrossinginterrupttime = 0;
        zerocrossinginterrupttime_d = 0;
        phaseinterrupttime = 0;
        phaseinterrupttime_d = 0;

        OCR1Anew = OCR1A_DEFAULT;

        phase = STEPS_PER_CYCLE - 1;

        timedifference = 0;
        timedifference_d = 0;
        integrator = 0;
        error = 0;
        derivative = 0;
        crossed = -1;
        synclevel = stepsneededforsync;
        insync = false;

        pinMode(zerocrossing_interruptpin, INPUT);
        attachInterrupt(digitalPinToInterrupt(zerocrossing_interruptpin), JPDimmerLib::zeroCrossingInterrupt, RISING);
        synccall = synccall_;
        synccall(0);

        lamps = (lamp *)malloc(AMOUNT_OF_LAMPS * sizeof(lamp));

        for (int i = 0; i < AMOUNT_OF_LAMPS; i++)
        {
                lamps[i].addresstoprogram = NULL;
        }

        burninglampoverview = 0;
        fadingoverview = 0;
}

uint8_t JPDimmerLib::getrequestedbrightness(uint8_t lampnumber)
{
        return lamps[lampnumber].requested_brightness;
}

uint8_t JPDimmerLib::getactualbrightness(uint8_t lampnumber)
{
        return lamps[lampnumber].brightness;
}

boolean JPDimmerLib::getfading(uint8_t lampnumber)
{
        return lamps[lampnumber].fading;
}

boolean JPDimmerLib::getinited(uint8_t lampnumber)
{
        return (lamps[lampnumber].addresstoprogram != NULL);
}

boolean JPDimmerLib::getsyncstatus()
{
        return insync;
}

boolean JPDimmerLib::getactivity()
{
        return !(burninglampoverview == 0 && fadingoverview == 0);
}

void JPDimmerLib::lampinit(uint8_t lampnumber, int pin)
{
        pinMode(pin, OUTPUT);

        lamps[lampnumber].bitinprogram = digitalPinToBitMask(pin);
        if (portOutputRegister(digitalPinToPort(pin)) == &PORTB)
        {
                lamps[lampnumber].addresstoprogram = portBprogram;
                lamps[lampnumber].addresstoreset = &portBresetmask;
                portBresetmaskinit &= ~lamps[lampnumber].bitinprogram;
                portBresetmask &= ~lamps[lampnumber].bitinprogram;
        }
        else if (portOutputRegister(digitalPinToPort(pin)) == &PORTC)
        {
                lamps[lampnumber].addresstoprogram = portCprogram;
                lamps[lampnumber].addresstoreset = &portCresetmask;
                portCresetmaskinit &= ~lamps[lampnumber].bitinprogram;
                portCresetmask &= ~lamps[lampnumber].bitinprogram;
        }
        else if (portOutputRegister(digitalPinToPort(pin)) == &PORTD)
        {
                lamps[lampnumber].addresstoprogram = portDprogram;
                lamps[lampnumber].addresstoreset = &portDresetmask;
                portDresetmaskinit &= ~lamps[lampnumber].bitinprogram;
                portDresetmask &= ~lamps[lampnumber].bitinprogram;
        }
        else
        {
                lamps[lampnumber].addresstoprogram = NULL;
        }
        lamps[lampnumber].setting = 0;
        lamps[lampnumber].brightness = 0;
        lamps[lampnumber].lastbrightness = 100;
        lamps[lampnumber].requested_brightness = 0;
        lamps[lampnumber].fading = false;
        lamps[lampnumber].addresstoclear = 0;
}

void JPDimmerLib::phasecorrection()
{
        timedifference = (int32_t)((int32_t)(zerocrossinginterrupttime+(uint32_t)ZEROCROSSING_OFFSET+((uint32_t)ZEROCROSSING_CYCLE_OFFSET*(uint32_t)AC_PERIOD / (uint32_t)STEPS_PER_CYCLE))-(int32_t)phaseinterrupttime);

        error = timedifference * kp;

        integrator += timedifference * ki;
        if (integrator > (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 4)))
                integrator = (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 4));
        if (integrator < -(AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 4)))
                integrator = -(AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 4));

        derivative = (timedifference - timedifference_d) * kd;

        OCR1Anew = OCR1A_DEFAULT + error + integrator + derivative;

        if (OCR1Anew > OCR1A_DEFAULT + (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 2)))
                OCR1Anew = OCR1A_DEFAULT + (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 2));
        if (OCR1Anew < OCR1A_DEFAULT - (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 2)))
                OCR1Anew = OCR1A_DEFAULT - (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 2));

        timedifference_d = timedifference;

        if (!(timedifference > (AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 1)) || timedifference < -(AC_PERIOD / ((int32_t)STEPS_PER_CYCLE * 1))))
        {
                if (synclevel)
                {
                        synclevel--;
                }
        }
        else
                synclevel = STEPS_NEEDED_FOR_SYNC;
}

void JPDimmerLib::ACperiodicstuff()
{
        if (synclevel == 0)
        {
                insync = true;
                if (synccall)
                        synccall(true);
        }
        else
        {
                insync = false;
                if (synccall)
                        synccall(false);
        }
        for (int i = 0; i < AMOUNT_OF_LAMPS; i++)
                fadeLoop(i);
}

void JPDimmerLib::loop()
{
        if (zerocrossinginterrupttime != zerocrossinginterrupttime_d)
        {
                zerocrossinginterrupttime_d = zerocrossinginterrupttime;
                if (crossed > STEPS_AFTER_TIMER_DISABLE)
                {
                        setuptimer1();
                        starttimer1();
                }
                crossed = 0;
        }

        if (phaseinterrupttime != phaseinterrupttime_d)
        {
                phaseinterrupttime_d = phaseinterrupttime;
                if (crossed > STEPS_AFTER_TIMER_DISABLE)
                {
                        stoptimer1();
                        disabletriacs();
                        synclevel = STEPS_NEEDED_FOR_SYNC;
                }
                else
                {
                        phasecorrection();
                }
                crossed++;
                ACperiodicstuff();
        }
}

boolean JPDimmerLib::requestbrightness(uint8_t lampnumber, uint8_t brightness) // this can fail (returns false then)!!
{
        int convertedbrightness;

        if (lamps[lampnumber].addresstoclear)
        {
                lamps[lampnumber].addresstoprogram[lamps[lampnumber].addresstoclear]  &= ~lamps[lampnumber].bitinprogram;
                lamps[lampnumber].addresstoclear = 0;
        }

        if (insync && brightness <= 100 && lamps[lampnumber].addresstoprogram)
        {
                if (brightness != lamps[lampnumber].brightness)
                {
                        if (brightness > minimum_brightness)
                                convertedbrightness = brightness * (STEPS_PER_CYCLE - 1) / 100;
                        else
                                convertedbrightness = 0;

                        if (convertedbrightness != lamps[lampnumber].setting)
                        {

                                if (convertedbrightness == 0)
                                {
                                        lamps[lampnumber].addresstoprogram[lamps[lampnumber].setting]  &= ~lamps[lampnumber].bitinprogram;
                                        burninglampoverview &= ~(1 << lampnumber);

                                }
                                else
                                {
                                        lamps[lampnumber].addresstoprogram[convertedbrightness] |= lamps[lampnumber].bitinprogram;
                                        burninglampoverview |= (1 << lampnumber);
                                        if (convertedbrightness > lamps[lampnumber].setting)
                                        {
                                                lamps[lampnumber].addresstoclear = lamps[lampnumber].setting;
                                        }
                                        else
                                        {
                                                lamps[lampnumber].addresstoprogram[lamps[lampnumber].setting]  &= ~lamps[lampnumber].bitinprogram;
                                        }
                                }
                                if (convertedbrightness == STEPS_PER_CYCLE - 1)
                                {
                                        lamps[lampnumber].addresstoreset[0] |= lamps[lampnumber].bitinprogram;
                                }
                                else
                                {
                                        lamps[lampnumber].addresstoreset[0] &= ~lamps[lampnumber].bitinprogram;
                                }

                                lamps[lampnumber].setting = convertedbrightness;
                        }
                        lamps[lampnumber].brightness = brightness;
                }
        }
        else
                return false;
        return true;
}

void JPDimmerLib::fadeto(uint8_t lampnumber, uint8_t brightness, uint16_t steps, unsigned long duration, unsigned long startdelay)
{
        if (brightness != lamps[lampnumber].requested_brightness &&
            lamps[lampnumber].addresstoprogram && brightness <= 100)
        {
                if (!brightness)
                        lamps[lampnumber].lastbrightness = lamps[lampnumber].brightness;

                lamps[lampnumber].fadesteps = steps;

                lamps[lampnumber].requested_brightness = brightness;

                lamps[lampnumber].start_brightness = lamps[lampnumber].brightness;

                // calculate the width of each step
                lamps[lampnumber].step = ((int32_t)(brightness - lamps[lampnumber].brightness) << 8) / steps;

                lamps[lampnumber].lastfadestep = 0;
                lamps[lampnumber].fading = true;
                fadingoverview |= (1<<lampnumber);
                lamps[lampnumber].fadedelay = duration / steps;
                lamps[lampnumber].fadestarttime = millis() + startdelay;
                lamps[lampnumber].fadelastsyncchecktime = millis();
        }
}

void JPDimmerLib::fadetolast(uint8_t lampnumber, uint16_t steps, unsigned long duration, unsigned long startdelay)
{
        if (!lamps[lampnumber].requested_brightness && !lamps[lampnumber].fading)
                fadeto(lampnumber, lamps[lampnumber].lastbrightness, steps, duration, startdelay);
}

void JPDimmerLib::setbrightness(uint8_t lampnumber, uint8_t brightness)
{
        fadeto(lampnumber, brightness, 1, 1, 0);
}

void JPDimmerLib::setlastbrightness(uint8_t lampnumber)
{
        if (!lamps[lampnumber].requested_brightness && !lamps[lampnumber].fading)
                fadeto(lampnumber, lamps[lampnumber].lastbrightness, 1, 1, 0);
}

void JPDimmerLib::fadeLoop(uint8_t lampnumber)
{
        uint16_t currentfadestep;

        if (lamps[lampnumber].addresstoprogram && lamps[lampnumber].fading)
        {
                if (insync)
                {
                        lamps[lampnumber].fadelastsyncchecktime = millis();
                        if ((signed long)(millis() - lamps[lampnumber].fadestarttime) > 0)
                        {
                                currentfadestep = ((unsigned long)(millis() - lamps[lampnumber].fadestarttime)) / lamps[lampnumber].fadedelay;
                                if (currentfadestep >= lamps[lampnumber].fadesteps)
                                {
                                        if (requestbrightness(lampnumber, lamps[lampnumber].requested_brightness))
                                        {
                                                lamps[lampnumber].fading = false;
                                                fadingoverview &= ~(1<<lampnumber);

                                        }
                                }
                                else if (currentfadestep != lamps[lampnumber].lastfadestep)
                                {
                                        if (requestbrightness(lampnumber, ((((int32_t)(lamps[lampnumber].start_brightness)) << 8) + (currentfadestep * lamps[lampnumber].step)) >> 8)) // target minus distance to target steps
                                        {
                                                //set brightness of current step
                                                lamps[lampnumber].lastfadestep = currentfadestep;
                                        }
                                }
                        }
                }
                else
                {
                        lamps[lampnumber].fadestarttime += (unsigned long)(millis() - lamps[lampnumber].fadelastsyncchecktime);
                        lamps[lampnumber].fadelastsyncchecktime = millis();
                }
        }
}

void JPDimmerLib::JPDimmerLib::setuptimer1()
{
        // set up Timer 1
        TCCR1A = 0;    // normal operation
        TCCR1B = 0; // no counting
        TCNT1  = 0;
        OCR1A = OCR1A_DEFAULT;
        OCR1Anew = OCR1A_DEFAULT;
        TIMSK1 = bit (OCIE1A);       // interrupt on Compare A Match
}

void JPDimmerLib::starttimer1()
{
        TCCR1B = bit(WGM12) | bit(CS10); // CTC, no pre-scaling
}

void JPDimmerLib::stoptimer1()
{
        TCCR1B = 0; // CTC, no pre-scaling
}

void JPDimmerLib::disabletriacs()
{
        PORTB &= portBresetmaskinit;
        PORTD &= portDresetmaskinit;
        PORTC &= portCresetmaskinit;
}
