# exSIDBlaster - a farewell gift

If you want, you can now convert your SIDBlaster into half an exSID. You may have already heard about its virtually perfect sound reproduction. After the conversion, you essentially have an exSID but only one SID. You can use this in JSIDPlay2 or the web version. Simply enable "Fake Stereo" and the sound will always play. Please note that the exSID firmware is under a Creative Commons license (http://hacks.slashdirt.org/contact.html), so I cannot offer you a conversion. Basically, you only need to break one wire and solder another as a bridge. Cut pin 7 of the PIC with a small flat side cutter (or perhaps nail clippers). 2.: Solder a jumper from PIC pin 15 to SID socket pin 5. Then flash the new firmware (hex file) and program the new template (make sure you choose the right one!). You can find out how to do this in the SIDBlaster assembly instructions and manual. 

Theoretically, a similar conversion might work with the SIDBlaster nano, but in practice I would advise against it: The smaller package size of the PIC might not be able to cope as well with the additional thermal load caused by the overclocking.

A hardsid.dll wrapper DLL for exSID is still in the works.



Andreas