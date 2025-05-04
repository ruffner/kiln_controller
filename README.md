# Kiln controller

This project started with the restoration and modernization of a Crusader 274S kiln that was in disrepair, with only one or two functional elements.

* insert kiln restoration pics and steps here *

Initially, [Saur0o0n's PIDKiln](https://github.com/Saur0o0n/PIDKiln) seemed perfect, so it was built and implemented. Since my kiln is not in WiFi range, this was not ideal. I wanted a longer range telemetry link, and also more GPIO than the esp32 offered (especially given the full utilization already present within PIDKiln. 

This led to the inspiration of a new project based on a Teensy 3.6 that was laying around. Plenty of GPIO and very convenient. PIDKiln also uses MAX3185's that aren't that don't have compensation built in for the increase in non-linearity that K type TC's have at temperatures over 1000 deg C. For this reason I went with a 4 channel MCP960* based board from [PlayingWithFusion](https://www.playingwithfusion.com/productview.php?pdid=120). I have laid out some PCBs for these chips before and I think they offer a nice product.

This is the code that runs on the teensy. Data is logged to and SD card for reviewing firing results. Profiles to fire are loaded from the card as well. There is also groundstation code that runs on a FeatherM0 SAMD21 board and interfaces with a PC for remote control and telem logging. 

Improvements to be made: 
* control the bottom element separately for more even temperature throughout the kiln
* Add ability to add and save a firing profile from the groundstation
* Screen / minimal UI with buttons to fire a preset without needing the groundstation.
* Remote download of logfiles from SD card to groundstation to avoid fiddling with a microSD card
* make a PCB for it
* make an enclosure for it
* oof
