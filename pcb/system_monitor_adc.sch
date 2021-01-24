EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RespiraWorks_Std:ADC128D818 U?
U 1 1 5FF978B8
P 7500 3200
F 0 "U?" H 7600 4050 50  0000 C CNN
F 1 "ADC128D818" H 7800 3950 50  0000 C CNN
F 2 "RespiraWorks_Std:TSSOP-16_4.4x5mm_P0.65mm" H 7500 3200 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/adc128d818.pdf" H 7500 3200 50  0001 C CNN
	1    7500 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2600 8900 2600
Wire Wire Line
	8000 2700 8900 2700
Text HLabel 8900 2600 2    50   BiDi ~ 0
SDA
Text HLabel 8900 2700 2    50   Input ~ 0
SCL
Wire Wire Line
	8000 3100 8500 3100
Text HLabel 8900 3100 2    50   Output ~ 0
nINT
Wire Wire Line
	4450 3500 4900 3500
Wire Wire Line
	7500 2400 7500 1400
Wire Wire Line
	7500 1400 7700 1400
Text HLabel 9150 1400 2    50   Input ~ 0
Vcc_3V3
Text Label 7500 1400 0    50   ~ 0
Vcc_3V3
Text Label 6200 3750 0    50   ~ 0
Vcc_3V3
Wire Wire Line
	3850 3500 3650 3500
Wire Wire Line
	3850 3600 3650 3600
Wire Wire Line
	3650 3600 3650 3500
Connection ~ 3650 3500
Wire Wire Line
	3650 3500 3050 3500
$Comp
L power:GND #PWR?
U 1 1 5FFDE2B2
P 4000 4200
F 0 "#PWR?" H 4000 3950 50  0001 C CNN
F 1 "GND" H 4005 4027 50  0000 C CNN
F 2 "" H 4000 4200 50  0001 C CNN
F 3 "" H 4000 4200 50  0001 C CNN
	1    4000 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FFE3E0A
P 3050 3900
AR Path="/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8AFE9D/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8FBD97/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8E0754/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8DFCAD/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8DEDC0/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8C8865/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5E8E1F08/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4DF5/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/5FFDC330/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/600A8E88/5FFE3E0A" Ref="C?"  Part="1" 
AR Path="/600BE696/5FFE3E0A" Ref="C?"  Part="1" 
F 0 "C?" H 3150 3700 50  0000 C CNN
F 1 "100nF 100V X7R" H 3400 3800 50  0000 C CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 3088 3750 50  0001 C CNN
F 3 "~" H 3050 3900 50  0001 C CNN
F 4 "" H 3050 3900 50  0001 C CNN "Manufacturer"
F 5 "" H 3050 3900 50  0001 C CNN "Part Number"
F 6 "DNP" H 3350 3700 50  0000 C CNN "DNP"
	1    3050 3900
	1    0    0    1   
$EndComp
$Comp
L Device:C C?
U 1 1 5FFE4201
P 4900 3900
AR Path="/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8AFE9D/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8FBD97/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8E0754/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8DFCAD/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8DEDC0/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8C8865/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5E8E1F08/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4DF5/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/5FFE4201" Ref="C?"  Part="1" 
AR Path="/5FFDC330/5FFE4201" Ref="C?"  Part="1" 
AR Path="/600A8E88/5FFE4201" Ref="C?"  Part="1" 
AR Path="/600BE696/5FFE4201" Ref="C?"  Part="1" 
F 0 "C?" H 5000 3700 50  0000 C CNN
F 1 "100nF 100V X7R" H 5250 3800 50  0000 C CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 4938 3750 50  0001 C CNN
F 3 "~" H 4900 3900 50  0001 C CNN
F 4 "" H 4900 3900 50  0001 C CNN "Manufacturer"
F 5 "" H 4900 3900 50  0001 C CNN "Part Number"
F 6 "DNP" H 5200 3700 50  0000 C CNN "DNP"
	1    4900 3900
	1    0    0    1   
$EndComp
Wire Wire Line
	4900 3750 4900 3500
Connection ~ 4900 3500
Wire Wire Line
	3050 3750 3050 3500
Connection ~ 3050 3500
Wire Wire Line
	3050 3500 2700 3500
Wire Wire Line
	3050 4050 3050 4200
Wire Wire Line
	3050 4200 4000 4200
Wire Wire Line
	4900 4200 4900 4050
Wire Wire Line
	4150 3900 4150 4200
Connection ~ 4150 4200
Wire Wire Line
	4150 4200 4900 4200
Connection ~ 4000 4200
Wire Wire Line
	4000 4200 4150 4200
$Comp
L Device:R R?
U 1 1 5FFE655B
P 6100 4550
AR Path="/5FCD4B8E/5FCD4BC5/5FFE655B" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFE655B" Ref="R?"  Part="1" 
AR Path="/5FCD4D85/5FFE655B" Ref="R?"  Part="1" 
AR Path="/5FFDC330/5FFE655B" Ref="R?"  Part="1" 
AR Path="/600A8E88/5FFE655B" Ref="R?"  Part="1" 
AR Path="/600BE696/5FFE655B" Ref="R?"  Part="1" 
F 0 "R?" H 5950 4500 50  0000 L CNN
F 1 "10K 1%" H 5750 4600 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 6030 4550 50  0001 C CNN
F 3 "~" H 6100 4550 50  0001 C CNN
	1    6100 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 5FFE6561
P 6550 3900
AR Path="/5FCD4B8E/5FCD4BC5/5FFE6561" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFE6561" Ref="R?"  Part="1" 
AR Path="/5FCD4D85/5FFE6561" Ref="R?"  Part="1" 
AR Path="/5FFDC330/5FFE6561" Ref="R?"  Part="1" 
AR Path="/600A8E88/5FFE6561" Ref="R?"  Part="1" 
AR Path="/600BE696/5FFE6561" Ref="R?"  Part="1" 
F 0 "R?" H 6400 3850 50  0000 L CNN
F 1 "DNP" H 6350 3950 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 6480 3900 50  0001 C CNN
F 3 "~" H 6550 3900 50  0001 C CNN
	1    6550 3900
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FFE656A
P 6100 4700
AR Path="/5FCD4B8E/5FCD4BC5/5FFE656A" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFE656A" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4D85/5FFE656A" Ref="#PWR?"  Part="1" 
AR Path="/5FFDC330/5FFE656A" Ref="#PWR?"  Part="1" 
AR Path="/600A8E88/5FFE656A" Ref="#PWR?"  Part="1" 
AR Path="/600BE696/5FFE656A" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6100 4450 50  0001 C CNN
F 1 "GND" H 6105 4527 50  0000 C CNN
F 2 "" H 6100 4700 50  0001 C CNN
F 3 "" H 6100 4700 50  0001 C CNN
	1    6100 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FFE6570
P 6550 4700
AR Path="/5FCD4B8E/5FCD4BC5/5FFE6570" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFE6570" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4D85/5FFE6570" Ref="#PWR?"  Part="1" 
AR Path="/5FFDC330/5FFE6570" Ref="#PWR?"  Part="1" 
AR Path="/600A8E88/5FFE6570" Ref="#PWR?"  Part="1" 
AR Path="/600BE696/5FFE6570" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6550 4450 50  0001 C CNN
F 1 "GND" H 6555 4527 50  0000 C CNN
F 2 "" H 6550 4700 50  0001 C CNN
F 3 "" H 6550 4700 50  0001 C CNN
	1    6550 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FFE733F
P 6550 4550
AR Path="/5FCD4B8E/5FCD4BC5/5FFE733F" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFE733F" Ref="R?"  Part="1" 
AR Path="/5FCD4D85/5FFE733F" Ref="R?"  Part="1" 
AR Path="/5FFDC330/5FFE733F" Ref="R?"  Part="1" 
AR Path="/600A8E88/5FFE733F" Ref="R?"  Part="1" 
AR Path="/600BE696/5FFE733F" Ref="R?"  Part="1" 
F 0 "R?" H 6400 4500 50  0000 L CNN
F 1 "10K 1%" H 6200 4600 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 6480 4550 50  0001 C CNN
F 3 "~" H 6550 4550 50  0001 C CNN
	1    6550 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	7000 3800 7000 4250
Wire Wire Line
	7000 4250 6550 4250
Wire Wire Line
	6550 4250 6550 4400
Wire Wire Line
	7000 3700 6900 3700
Wire Wire Line
	6900 3700 6900 4150
Wire Wire Line
	6100 4150 6100 4400
$Comp
L Device:R R?
U 1 1 5FFE8713
P 6100 3900
AR Path="/5FCD4B8E/5FCD4BC5/5FFE8713" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFE8713" Ref="R?"  Part="1" 
AR Path="/5FCD4D85/5FFE8713" Ref="R?"  Part="1" 
AR Path="/5FFDC330/5FFE8713" Ref="R?"  Part="1" 
AR Path="/600A8E88/5FFE8713" Ref="R?"  Part="1" 
AR Path="/600BE696/5FFE8713" Ref="R?"  Part="1" 
F 0 "R?" H 5950 3850 50  0000 L CNN
F 1 "DNP" H 5900 3950 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 6030 3900 50  0001 C CNN
F 3 "~" H 6100 3900 50  0001 C CNN
	1    6100 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4900 3500 5050 3500
Wire Wire Line
	6900 4150 6100 4150
Wire Wire Line
	6100 4050 6100 4150
Connection ~ 6100 4150
Wire Wire Line
	6550 4050 6550 4250
Connection ~ 6550 4250
Wire Wire Line
	6550 3750 6100 3750
Text HLabel 2700 3500 0    50   Input ~ 0
Vcc_5V
$Comp
L Device:C C?
U 1 1 5FFF0376
P 7700 1700
AR Path="/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8AFE9D/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8FBD97/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8E0754/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8DFCAD/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8DEDC0/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8C8865/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5E8E1F08/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4DF5/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/5FFF0376" Ref="C?"  Part="1" 
AR Path="/5FFDC330/5FFF0376" Ref="C?"  Part="1" 
AR Path="/600A8E88/5FFF0376" Ref="C?"  Part="1" 
AR Path="/600BE696/5FFF0376" Ref="C?"  Part="1" 
F 0 "C?" H 7800 1500 50  0000 C CNN
F 1 "100nF 100V X7R" H 8050 1600 50  0000 C CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 7738 1550 50  0001 C CNN
F 3 "~" H 7700 1700 50  0001 C CNN
F 4 "" H 7700 1700 50  0001 C CNN "Manufacturer"
F 5 "" H 7700 1700 50  0001 C CNN "Part Number"
	1    7700 1700
	1    0    0    1   
$EndComp
Wire Wire Line
	7700 1550 7700 1400
Connection ~ 7700 1400
Wire Wire Line
	7700 1400 8550 1400
$Comp
L power:GND #PWR?
U 1 1 5FFF1F14
P 7700 1850
F 0 "#PWR?" H 7700 1600 50  0001 C CNN
F 1 "GND" H 7705 1677 50  0000 C CNN
F 2 "" H 7700 1850 50  0001 C CNN
F 3 "" H 7700 1850 50  0001 C CNN
	1    7700 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FFF6D49
P 7500 4000
F 0 "#PWR?" H 7500 3750 50  0001 C CNN
F 1 "GND" H 7505 3827 50  0000 C CNN
F 2 "" H 7500 4000 50  0001 C CNN
F 3 "" H 7500 4000 50  0001 C CNN
	1    7500 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FFF7298
P 8500 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FFF7298" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FFF7298" Ref="R?"  Part="1" 
AR Path="/5FCD4D85/5FFF7298" Ref="R?"  Part="1" 
AR Path="/5FFDC330/5FFF7298" Ref="R?"  Part="1" 
AR Path="/600A8E88/5FFF7298" Ref="R?"  Part="1" 
AR Path="/600BE696/5FFF7298" Ref="R?"  Part="1" 
F 0 "R?" H 8350 2900 50  0000 L CNN
F 1 "10K 1%" H 8150 3000 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 8430 2950 50  0001 C CNN
F 3 "~" H 8500 2950 50  0001 C CNN
	1    8500 2950
	-1   0    0    1   
$EndComp
Connection ~ 8500 3100
Wire Wire Line
	8500 3100 8900 3100
Text Label 8850 2800 2    50   ~ 0
Vcc_3V3
Wire Wire Line
	8500 2800 8850 2800
$Comp
L Device:C C?
U 1 1 5FFF86CD
P 8550 1700
AR Path="/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8AFE9D/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8FBD97/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8E0754/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8DFCAD/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8DEDC0/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8C8865/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5E8E1F08/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4DF5/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F5EF3/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F7154/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F719E/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F71F1/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/600F723B/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FCD4EEA/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/5FFDC330/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/600A8E88/5FFF86CD" Ref="C?"  Part="1" 
AR Path="/600BE696/5FFF86CD" Ref="C?"  Part="1" 
F 0 "C?" H 8650 1500 50  0000 C CNN
F 1 "1uF 16V X5R" H 8900 1600 50  0000 C CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 8588 1550 50  0001 C CNN
F 3 "~" H 8550 1700 50  0001 C CNN
F 4 "" H 8550 1700 50  0001 C CNN "Manufacturer"
F 5 "" H 8550 1700 50  0001 C CNN "Part Number"
	1    8550 1700
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FFF8AE5
P 8550 1850
F 0 "#PWR?" H 8550 1600 50  0001 C CNN
F 1 "GND" H 8555 1677 50  0000 C CNN
F 2 "" H 8550 1850 50  0001 C CNN
F 3 "" H 8550 1850 50  0001 C CNN
	1    8550 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 1550 8550 1400
Connection ~ 8550 1400
Wire Wire Line
	8550 1400 9150 1400
Text HLabel 7000 2600 0    50   Input ~ 0
ADC0
Text HLabel 7000 2700 0    50   Input ~ 0
ADC1
Text HLabel 7000 2800 0    50   Input ~ 0
ADC2
Text HLabel 7000 2900 0    50   Input ~ 0
ADC3
Text HLabel 7000 3000 0    50   Input ~ 0
ADC4
Text HLabel 7000 3100 0    50   Input ~ 0
ADC5
Text HLabel 7000 3200 0    50   Input ~ 0
ADC6
Text HLabel 7000 3300 0    50   Input ~ 0
ADC7
$Comp
L RespiraWorks_Std:LM4120-IM5X-3.3 U?
U 1 1 5FFDD58B
P 4150 3600
F 0 "U?" H 4150 3967 50  0000 C CNN
F 1 "LM4120-IM5X-3.3" H 4150 3876 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4200 3350 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm4128.pdf" V 4150 3700 50  0001 C CIN
F 4 "DNP" H 4350 3950 50  0000 C CNN "DNP"
	1    4150 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 600AAE9A
P 5050 3200
AR Path="/5FCD4B8E/5FCD4BC5/600AAE9A" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/600AAE9A" Ref="R?"  Part="1" 
AR Path="/5FCD4D85/600AAE9A" Ref="R?"  Part="1" 
AR Path="/5FFDC330/600AAE9A" Ref="R?"  Part="1" 
AR Path="/600A8E88/600AAE9A" Ref="R?"  Part="1" 
AR Path="/600BE696/600AAE9A" Ref="R?"  Part="1" 
F 0 "R?" H 4850 3150 50  0000 L CNN
F 1 "0" H 4900 3250 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 4980 3200 50  0001 C CNN
F 3 "~" H 5050 3200 50  0001 C CNN
	1    5050 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	5050 3350 5050 3500
Connection ~ 5050 3500
Wire Wire Line
	5050 3500 7000 3500
Wire Wire Line
	5050 3050 4650 3050
Text Label 4650 3050 0    50   ~ 0
Vcc_3V3
$EndSCHEMATC