# ULP Sound Player
## Introduction
These sketch show how to use the Ultra Low Power coprocessor (U.L.P.) of the ESP32 in order to play music, and relieve main processor's core of this task. Only a lightweight task refill from time to time the ULP separate memory with instruction which contains samples. That could be usefull for videos games, where graphics can monopolize both 2 cores.

This awesome video explain how it works :
https://www.youtube.com/watch?v=6PGrsZmYAJ0

## Setup

Just connect your speaker to pin 25 if you use the mono mode, or connect left speaker to pin 25 and right one to pin 26. Then, just upload the corresponding sketch to your ESP32.

## Convert Your Sound

Sample array can be easily replaced with your sound :

- You can use this online converter which is straightforward : https://bitluni.net/wp-content/uploads/2018/03/WavetableEditor.html

- Or use audacity & an hexadecimal editor for tuning and a better undestanding of how it works.

Be carefull, ESP32 has only 4 MB of Flash, which can contain only few seconds of sound (depending of the sampling rate, stereo/mono mode etc...).

