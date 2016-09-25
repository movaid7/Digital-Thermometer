# Digital-Thermometer
A digital thermometer (range 0-99℃) with an audio alarm that sounds if temperature exceeds user-set bounds


The project was done in PIC assembly and implemented on hardware. 

A PIC16F690 chip was used, with the temperature value being displayed on a pair of multiplexed common cathode seven-segment displays. Two buttons were used as up and down switches, to adjust the temperarure threshold. If this value was exceeeded, a 440Hz frequency was played on a buzzer.
