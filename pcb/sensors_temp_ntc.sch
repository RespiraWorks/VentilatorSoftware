EESchema Schematic File Version 4
LIBS:Ventilator-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 23 28
Title "NTC Temperature Sensor Input"
Date ""
Rev "2.0"
Comp "RespiraWorks"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 6100 3300 2    50   Output ~ 0
Out
Text HLabel 4100 2550 0    50   Input ~ 0
Vcc
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 6010AD26
P 1550 3200
AR Path="/5FCD4EEA/600F5EF3/6010AD26" Ref="J?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/6010AD26" Ref="J?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/6010AD26" Ref="J?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/6010AD26" Ref="J?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/6010AD26" Ref="J?"  Part="1" 
F 0 "J?" H 1468 3417 50  0000 C CNN
F 1 "Molex Micro-Fit" H 1468 3326 50  0000 C CNN
F 2 "RespiraWorks_Std:Molex_Micro-Fit_3.0_43650-0215_1x02_P3.00mm_Vertical" H 1550 3200 50  0001 C CNN
F 3 "~" H 1550 3200 50  0001 C CNN
	1    1550 3200
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6010B42D
P 4300 3850
AR Path="/5FCD4B8E/5FCD4BC5/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BF6/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/606EF10F/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6007E2BB/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/60073FCF/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/6010B42D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/6010B42D" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4300 3600 50  0001 C CNN
F 1 "GND" H 4305 3677 50  0000 C CNN
F 2 "" H 4300 3850 50  0001 C CNN
F 3 "" H 4300 3850 50  0001 C CNN
	1    4300 3850
	-1   0    0    -1  
$EndComp
Text Notes 1000 3300 0    50   ~ 0
PT100 RTD
$Comp
L Device:C C?
U 1 1 60117C61
P 5600 3450
AR Path="/5FCD4B8E/5FCD4BC5/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BF6/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/606EF10F/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/6007E2BB/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/60073FCF/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/600630A6/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/6007AC3B/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/6007DA64/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4E9D/6007BD34/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/60117C61" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/60117C61" Ref="C?"  Part="1" 
F 0 "C?" H 5900 3450 50  0000 L CNN
F 1 "4.7uF 10V X5R" H 5700 3550 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 5638 3300 50  0001 C CNN
F 3 "~" H 5600 3450 50  0001 C CNN
	1    5600 3450
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60117C67
P 5600 3850
AR Path="/5FCD4B8E/5FCD4BC5/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BF6/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/606EF10F/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6007E2BB/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/60073FCF/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/600630A6/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6007AC3B/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6007DA64/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6007BD34/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/60117C67" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/60117C67" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5600 3600 50  0001 C CNN
F 1 "GND" H 5605 3677 50  0000 C CNN
F 2 "" H 5600 3850 50  0001 C CNN
F 3 "" H 5600 3850 50  0001 C CNN
	1    5600 3850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5600 3300 6100 3300
Text Notes 5200 4600 0    50   ~ 0
LPF varies with temp\n(60hz at 150C, 3hz at 25C, 5hz at 0C)\nOK because of large margin between ADC \nsampling rate and desired sensor bandwidth
Connection ~ 5600 3300
Text Notes 6800 3500 0    50   ~ 0
for 10k NTC B=3380K:\n@150C: R = 350       Vout = 3.12V   Sensitivity: 2mV/degC\n@100C: R = 1022     Vout = 2.94V   Sensitivity: 6mV/degC\n@25C:  R = 10000   Vout = 1.63V   Sensitivity: 30mV/degC\n@0C:    R = 28255   Vout = 0.86V   Sensitivity: 29mV/degC\n\nTemp (C) = B / ( ln( Vcc*10k/Vout/Rlim - (10k+220)/Rlim )  ) + 273.15\nWhere Rlim = R0*exp(-B/T0) and T0 is in kelvin.
Text Notes 3250 2300 0    50   ~ 0
NTC placed on the high side of divider so that\nNTC missing or break does not drive ADC input to VCC\n\nPeak self-heating is 0.2mW, glass bead thermistor in air \nis about 1.5mW/degC, so this is a max error of 0.13degC \ndue to self-heating.
Wire Wire Line
	4100 2550 4300 2550
$Comp
L Device:R R?
U 1 1 60108F2B
P 4300 2700
AR Path="/60108F2B" Ref="R?"  Part="1" 
AR Path="/5E8AFE9D/60108F2B" Ref="R?"  Part="1" 
AR Path="/5E8C8865/60108F2B" Ref="R?"  Part="1" 
AR Path="/5E8E1F08/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4DF5/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4E9D/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4E9D/6007E2BB/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4E9D/60073FCF/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/60108F2B" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/60108F2B" Ref="R?"  Part="1" 
F 0 "R?" H 4150 2700 50  0000 C CNN
F 1 "220 0.1%" H 4000 2800 50  0000 C CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 4230 2700 50  0001 C CNN
F 3 "~" H 4300 2700 50  0001 C CNN
F 4 "KOA Speer" H 4300 2700 50  0001 C CNN "Manufacturer"
F 5 "RK73H2BTTD1004F" H 4300 2700 50  0001 C CNN "Part Number"
	1    4300 2700
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5FFCC04E
P 4300 3600
AR Path="/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5E8AFE9D/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5E8C8865/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5E8E1F08/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4DF5/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4E9D/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4E9D/6007E2BB/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4E9D/60073FCF/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFCC04E" Ref="R?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFCC04E" Ref="R?"  Part="1" 
F 0 "R?" H 4150 3600 50  0000 C CNN
F 1 "10K 0.1%" H 4000 3700 50  0000 C CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 4230 3600 50  0001 C CNN
F 3 "~" H 4300 3600 50  0001 C CNN
F 4 "KOA Speer" H 4300 3600 50  0001 C CNN "Manufacturer"
F 5 "RK73H2BTTD1004F" H 4300 3600 50  0001 C CNN "Part Number"
	1    4300 3600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 3300 2300 3300
Wire Wire Line
	4300 3300 4300 3450
Wire Wire Line
	1750 3200 4300 3200
Wire Wire Line
	4300 3200 4300 2850
Text Notes 3500 2700 0    50   ~ 0
Short current limit
Wire Wire Line
	4300 3750 4300 3850
Wire Wire Line
	4300 3300 5600 3300
Connection ~ 4300 3300
Wire Wire Line
	5600 3600 5600 3850
$Comp
L Power_Protection:USBLC6-2SC6 U?
U 1 1 5FFD5C58
P 2400 3900
AR Path="/5FCD4EEA/600F5EF3/5FFD5C58" Ref="U?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFD5C58" Ref="U?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFD5C58" Ref="U?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFD5C58" Ref="U?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFD5C58" Ref="U?"  Part="1" 
F 0 "U?" V 1950 4100 50  0000 L CNN
F 1 "USBLC6-2SC6" V 2050 4100 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 2400 3400 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/usblc6-2.pdf" H 2600 4250 50  0001 C CNN
	1    2400 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 3500 2500 3300
Wire Wire Line
	2500 3300 4300 3300
Wire Wire Line
	2300 3500 2300 3300
NoConn ~ 2300 4300
NoConn ~ 2500 4300
Wire Wire Line
	2800 3900 3100 3900
Text Label 2850 3900 0    50   ~ 0
Vcc
Text Label 4150 2550 0    50   ~ 0
Vcc
$Comp
L power:GND #PWR?
U 1 1 5FFDA1EB
P 1800 4050
AR Path="/5FCD4B8E/5FCD4BC5/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BF6/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/606EF10F/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/6007E2BB/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4E9D/60073FCF/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFDA1EB" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFDA1EB" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 1800 3800 50  0001 C CNN
F 1 "GND" H 1805 3877 50  0000 C CNN
F 2 "" H 1800 4050 50  0001 C CNN
F 3 "" H 1800 4050 50  0001 C CNN
	1    1800 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2000 3900 1800 3900
Wire Wire Line
	1800 3900 1800 4050
$EndSCHEMATC