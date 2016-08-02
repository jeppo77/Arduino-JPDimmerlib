# arduino-triacdimmer
A dimmer lib for arduino to be used to control a TRIAC.

Based on the zerocrossing (incoming interrupt), the pulses for the TRIAC(s) are generated. Several TRIACs ("lamps") can be controlled.
The zerocrossing detection is based on a PID controller. Timer 1 is used to generate the timing.

stepspercycle = amount of dimmer steps between 2 zerocrossings (for example 100) 
amountoflamps = amount of lamps to be supported
zerocrossing_interruptpin = zerocrossing interrupt pin
synccall = function to be called when going in or out of zerocrossing sync (eg to set a led)
minimumbrightness = minimal settable brightness (might be needed to make sure the pulse timing is ok for the TRIAC)
zerocrossingoffset = shifts the whole pulse generation timing related to the zerocrossing (might be needed to make sure the pulse timing is ok for the TRIAC)
stepsaftertimerdisable = out of sync steps needed after which timer 1 will be disabled
stepsneededforsync = in sync steps needed before in sync is flagged
tu, ku = PID controller setting for zerocrossing detection

lampinit(...) = initializes a lamp (start from lamp 0)
fadeto(...) = fades a lamp to a certain brightness in "steps" steps during "duration" after a startdelay
setbrightness(...) = set immediate brightness
loop() = to be called in arduino's main loop

I advice to use JPDimmer_Create to create the object


Tested on a 328P
