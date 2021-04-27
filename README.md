# DALI
Simple DALI controller using Arduino
To control DALI lighting fixtures I created DALI library. You can use it to work with DALI devices. At the moment you can not get responses from devices. But you can find whether devices give response or not. It's enough to control and initialize DALI luminaires.
https://create.arduino.cc/projecthub/NabiyevTR/simple-dali-driver-506e44


Links:
https://en.wikipedia.org/wiki/Digital_Addressable_Lighting_Interface



12 >> 5
    5 / 12 = 0.4
    20/(20+30) = 0.4

12 >> 3.3v
    3.3 / 12 = 0.275
    10/(10+30) = 0.25


R = U/I = (5 — 3.3 — 0.6)/500E-6 = 2.2 кОм

R1 и R2. wifi модуль подключаем параллельно резистору R1. Номиналы выбираем так, чтобы:
A) R1/(R1 + R2) = 3.3/5