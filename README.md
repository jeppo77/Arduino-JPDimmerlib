# arduino-triacdimmer
A dimmer lib for arduino to be used to control a TRIAC.

Based on the zerocrossing (incoming interrupt), the pulses for the TRIAC(s) are generated. Several TRIACs ("lamps") can be controlled.
The zerocrossing detection is based on a PID controller. Timer 1 is used to generate the timing.<br>

stepspercycle = amount of dimmer steps between 2 zerocrossings (for example 100) <br>
amountoflamps = amount of lamps to be supported<br>
zerocrossing_interruptpin = zerocrossing interrupt pin<br>
synccall = function to be called when going in or out of zerocrossing sync (eg to set a led)<br>
minimumbrightness = minimal settable brightness (might be needed to make sure the pulse timing is ok for the TRIAC)<br>
zerocrossingoffset = shifts the whole pulse generation timing related to the zerocrossing (might be needed to make sure the pulse timing is ok for the TRIAC)<br>
stepsaftertimerdisable = out of sync steps needed after which timer 1 will be disabled<br>
stepsneededforsync = in sync steps needed before in sync is flagged<br>
tu, ku = PID controller setting for zerocrossing detection<br>
<br>
lampinit(...) = initializes a lamp (start from lamp 0)<br>
fadeto(...) = fades a lamp to a certain brightness in "steps" steps during "duration" after a startdelay<br>
setbrightness(...) = set immediate brightness<br>
loop() = to be called in arduino's main loop<br>
<br>
I advice to use JPDimmer_Create to create the object<br>


Tested on a 328P<br>
