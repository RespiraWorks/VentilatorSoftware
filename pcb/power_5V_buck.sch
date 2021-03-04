EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 38
Title "5V Buck Converter"
Date ""
Rev "A"
Comp "RespiraWorks"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RespiraWorks:TPS54561 U?
U 1 1 5FD565BA
P 6300 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FD565BA" Ref="U?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD565BA" Ref="U?"  Part="1" 
F 0 "U?" H 6275 3617 50  0000 C CNN
F 1 "TPS54561" H 6275 3526 50  0000 C CNN
F 2 "RespiraWorks:WSON-10-1EP_4x4mm_P0.8mm_EP2.6x3mm" H 6100 2450 50  0001 L CIN
F 3 "" H 6050 3150 50  0001 C CNN
	1    6300 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FD56FAB
P 5050 3750
AR Path="/5FCD4B8E/5FCD4BC5/5FD56FAB" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD56FAB" Ref="R?"  Part="1" 
F 0 "R?" H 4800 3800 50  0000 L CNN
F 1 "249k" H 4800 3700 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 4980 3750 50  0001 C CNN
F 3 "~" H 5050 3750 50  0001 C CNN
	1    5050 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD57D47
P 6300 3600
AR Path="/5FCD4B8E/5FCD4BC5/5FD57D47" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD57D47" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6300 3350 50  0001 C CNN
F 1 "GND" H 6305 3427 50  0000 C CNN
F 2 "" H 6300 3600 50  0001 C CNN
F 3 "" H 6300 3600 50  0001 C CNN
	1    6300 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD5821D
P 5050 3900
AR Path="/5FCD4B8E/5FCD4BC5/5FD5821D" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD5821D" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5050 3650 50  0001 C CNN
F 1 "GND" H 5055 3727 50  0000 C CNN
F 2 "" H 5050 3900 50  0001 C CNN
F 3 "" H 5050 3900 50  0001 C CNN
	1    5050 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3600 6300 3600
Connection ~ 6300 3600
$Comp
L Device:R R?
U 1 1 5FD5C5C2
P 5650 3950
AR Path="/5FCD4B8E/5FCD4BC5/5FD5C5C2" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD5C5C2" Ref="R?"  Part="1" 
F 0 "R?" H 5400 4000 50  0000 L CNN
F 1 "16.9k" H 5350 3900 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 5580 3950 50  0001 C CNN
F 3 "~" H 5650 3950 50  0001 C CNN
	1    5650 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD5C5C8
P 5650 4400
AR Path="/5FCD4B8E/5FCD4BC5/5FD5C5C8" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD5C5C8" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5650 4150 50  0001 C CNN
F 1 "GND" H 5655 4227 50  0000 C CNN
F 2 "" H 5650 4400 50  0001 C CNN
F 3 "" H 5650 4400 50  0001 C CNN
	1    5650 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FD5CF8C
P 5650 4250
AR Path="/5FCD4B8E/5FCD4BC5/5FD5CF8C" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD5CF8C" Ref="C?"  Part="1" 
F 0 "C?" H 5350 4250 50  0000 L CNN
F 1 "4.7nF" H 5350 4150 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 5688 4100 50  0001 C CNN
F 3 "~" H 5650 4250 50  0001 C CNN
	1    5650 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FD5D9F4
P 5950 4250
AR Path="/5FCD4B8E/5FCD4BC5/5FD5D9F4" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD5D9F4" Ref="C?"  Part="1" 
F 0 "C?" H 6065 4296 50  0000 L CNN
F 1 "47pF" H 6065 4205 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 5988 4100 50  0001 C CNN
F 3 "~" H 5950 4250 50  0001 C CNN
	1    5950 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4400 5950 4400
Connection ~ 5650 4400
Wire Wire Line
	5950 4100 5950 3800
Wire Wire Line
	5650 3400 5650 3800
Wire Wire Line
	5050 3600 5050 3200
Wire Wire Line
	5050 3200 5650 3200
Connection ~ 5650 3800
Wire Wire Line
	5650 3800 5950 3800
Text Notes 4650 4200 0    50   ~ 0
~~400 kHz fsw
$Comp
L Device:C C?
U 1 1 5FD63EEB
P 4400 3700
AR Path="/5FCD4B8E/5FCD4BC5/5FD63EEB" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD63EEB" Ref="C?"  Part="1" 
F 0 "C?" H 4100 3700 50  0000 L CNN
F 1 "10nF" H 4050 3600 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 4438 3550 50  0001 C CNN
F 3 "~" H 4400 3700 50  0001 C CNN
	1    4400 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD6440A
P 4400 3850
AR Path="/5FCD4B8E/5FCD4BC5/5FD6440A" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD6440A" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4400 3600 50  0001 C CNN
F 1 "GND" H 4405 3677 50  0000 C CNN
F 2 "" H 4400 3850 50  0001 C CNN
F 3 "" H 4400 3850 50  0001 C CNN
	1    4400 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3550 4400 3000
$Comp
L Device:R R?
U 1 1 5FD66B59
P 4000 3250
AR Path="/5FCD4B8E/5FCD4BC5/5FD66B59" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD66B59" Ref="R?"  Part="1" 
F 0 "R?" H 3750 3300 50  0000 L CNN
F 1 "52.3k" H 3650 3200 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 3930 3250 50  0001 C CNN
F 3 "~" H 4000 3250 50  0001 C CNN
	1    4000 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FD66F4F
P 4000 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FD66F4F" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD66F4F" Ref="R?"  Part="1" 
F 0 "R?" H 3750 3000 50  0000 L CNN
F 1 "316k" H 3650 2900 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 3930 2950 50  0001 C CNN
F 3 "~" H 4000 2950 50  0001 C CNN
	1    4000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2800 4250 2800
Wire Wire Line
	4250 2800 4250 3100
Connection ~ 4000 3100
$Comp
L power:GND #PWR?
U 1 1 5FD69910
P 4000 3400
AR Path="/5FCD4B8E/5FCD4BC5/5FD69910" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD69910" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4000 3150 50  0001 C CNN
F 1 "GND" H 4005 3227 50  0000 C CNN
F 2 "" H 4000 3400 50  0001 C CNN
F 3 "" H 4000 3400 50  0001 C CNN
	1    4000 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2800 4000 2550
Wire Wire Line
	4400 3000 5650 3000
Wire Wire Line
	4000 2550 5650 2550
Wire Wire Line
	4250 3100 4000 3100
$Comp
L Device:C C?
U 1 1 5FD75E95
P 7150 2550
AR Path="/5FCD4B8E/5FCD4BC5/5FD75E95" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD75E95" Ref="C?"  Part="1" 
F 0 "C?" V 6900 2500 50  0000 L CNN
F 1 "100nF" V 7000 2300 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 7188 2400 50  0001 C CNN
F 3 "~" H 7150 2550 50  0001 C CNN
	1    7150 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	6900 2550 7000 2550
Wire Wire Line
	6900 2800 7300 2800
Wire Wire Line
	7300 2800 7300 2550
$Comp
L Device:R R?
U 1 1 5FD77C21
P 9700 3250
AR Path="/5FCD4B8E/5FCD4BC5/5FD77C21" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD77C21" Ref="R?"  Part="1" 
F 0 "R?" H 9450 3300 50  0000 L CNN
F 1 "52.3k" H 9350 3200 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 9630 3250 50  0001 C CNN
F 3 "~" H 9700 3250 50  0001 C CNN
	1    9700 3250
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FD77FDA
P 9700 3550
AR Path="/5FCD4B8E/5FCD4BC5/5FD77FDA" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD77FDA" Ref="R?"  Part="1" 
F 0 "R?" H 9450 3550 50  0000 L CNN
F 1 "10k" H 9450 3450 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 9630 3550 50  0001 C CNN
F 3 "~" H 9700 3550 50  0001 C CNN
	1    9700 3550
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD78374
P 9700 3700
AR Path="/5FCD4B8E/5FCD4BC5/5FD78374" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD78374" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 9700 3450 50  0001 C CNN
F 1 "GND" H 9705 3527 50  0000 C CNN
F 2 "" H 9700 3700 50  0001 C CNN
F 3 "" H 9700 3700 50  0001 C CNN
	1    9700 3700
	-1   0    0    -1  
$EndComp
$Comp
L Device:L L?
U 1 1 5FD7E9EB
P 7650 2800
AR Path="/5FCD4B8E/5FCD4BC5/5FD7E9EB" Ref="L?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD7E9EB" Ref="L?"  Part="1" 
F 0 "L?" V 7840 2800 50  0000 C CNN
F 1 "7.2uH" V 7749 2800 50  0000 C CNN
F 2 "RespiraWorks_Std:L_Wuerth_WE-PDF" H 7650 2800 50  0001 C CNN
F 3 "~" H 7650 2800 50  0001 C CNN
	1    7650 2800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7500 2800 7400 2800
Connection ~ 7300 2800
$Comp
L Device:C C?
U 1 1 5FD80839
P 8200 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FD80839" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD80839" Ref="C?"  Part="1" 
F 0 "C?" H 8150 2700 50  0000 L CNN
F 1 "100nF" H 8100 2600 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 8238 2800 50  0001 C CNN
F 3 "~" H 8200 2950 50  0001 C CNN
	1    8200 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:C C?
U 1 1 5FD8163B
P 8750 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FD8163B" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD8163B" Ref="C?"  Part="1" 
F 0 "C?" H 8700 2700 50  0000 L CNN
F 1 "1uF" H 8700 2600 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 8788 2800 50  0001 C CNN
F 3 "~" H 8750 2950 50  0001 C CNN
F 4 "AVX" H 8750 2950 50  0001 C CNN "Manufacturer"
	1    8750 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:C C?
U 1 1 5FD86508
P 9300 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FD86508" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD86508" Ref="C?"  Part="1" 
F 0 "C?" H 9250 2700 50  0000 L CNN
F 1 "47uF" H 9200 2600 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1210_3225Metric" H 9338 2800 50  0001 C CNN
F 3 "~" H 9300 2950 50  0001 C CNN
F 4 "Murata" H 9300 2950 50  0001 C CNN "Manufacturer"
F 5 "GRM32ER71A476KE15L" H 9300 2950 50  0001 C CNN "Manufacturer PN"
	1    9300 2950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD86A63
P 9300 3100
AR Path="/5FCD4B8E/5FCD4BC5/5FD86A63" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD86A63" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 9300 2850 50  0001 C CNN
F 1 "GND" H 9305 2927 50  0000 C CNN
F 2 "" H 9300 3100 50  0001 C CNN
F 3 "" H 9300 3100 50  0001 C CNN
	1    9300 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD86DE3
P 8750 3100
AR Path="/5FCD4B8E/5FCD4BC5/5FD86DE3" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD86DE3" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 8750 2850 50  0001 C CNN
F 1 "GND" H 8755 2927 50  0000 C CNN
F 2 "" H 8750 3100 50  0001 C CNN
F 3 "" H 8750 3100 50  0001 C CNN
	1    8750 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD870B2
P 8200 3100
AR Path="/5FCD4B8E/5FCD4BC5/5FD870B2" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD870B2" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 8200 2850 50  0001 C CNN
F 1 "GND" H 8205 2927 50  0000 C CNN
F 2 "" H 8200 3100 50  0001 C CNN
F 3 "" H 8200 3100 50  0001 C CNN
	1    8200 3100
	1    0    0    -1  
$EndComp
Connection ~ 9700 2800
Wire Wire Line
	7100 3100 6900 3100
Connection ~ 9700 3400
Text Label 7000 3100 0    50   ~ 0
Fb
Text HLabel 2100 2550 0    50   Input ~ 0
Vin
Text HLabel 10650 2800 2    50   Output ~ 0
Vout
$Comp
L Device:R R?
U 1 1 5FDA1BBA
P 10350 3100
AR Path="/5FCD4B8E/5FCD4BC5/5FDA1BBA" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDA1BBA" Ref="R?"  Part="1" 
F 0 "R?" H 10150 3150 50  0000 L CNN
F 1 "2k" H 10150 3050 50  0000 L CNN
F 2 "RespiraWorks_Std:R_0603_1608Metric" V 10280 3100 50  0001 C CNN
F 3 "~" H 10350 3100 50  0001 C CNN
	1    10350 3100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10350 2950 10350 2800
Connection ~ 10350 2800
Wire Wire Line
	10350 2800 10650 2800
$Comp
L Device:LED_ALT D?
U 1 1 5FDA4446
P 10350 3550
AR Path="/5FCD4B8E/5FCD4BC5/5FDA4446" Ref="D?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDA4446" Ref="D?"  Part="1" 
F 0 "D?" V 10389 3432 50  0000 R CNN
F 1 "Green" V 10298 3432 50  0000 R CNN
F 2 "RespiraWorks_Std:LED_0805_2012Metric" H 10350 3550 50  0001 C CNN
F 3 "~" H 10350 3550 50  0001 C CNN
	1    10350 3550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10350 3400 10350 3250
$Comp
L power:GND #PWR?
U 1 1 5FDA5398
P 10350 3700
AR Path="/5FCD4B8E/5FCD4BC5/5FDA5398" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDA5398" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 10350 3450 50  0001 C CNN
F 1 "GND" H 10355 3527 50  0000 C CNN
F 2 "" H 10350 3700 50  0001 C CNN
F 3 "" H 10350 3700 50  0001 C CNN
	1    10350 3700
	-1   0    0    -1  
$EndComp
Text Notes 10500 3750 0    50   ~ 0
~~1 mA on
$Comp
L Device:D_Schottky_AAK D?
U 1 1 5FDAE590
P 7400 2950
AR Path="/5FCD4B8E/5FCD4BC5/5FDAE590" Ref="D?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDAE590" Ref="D?"  Part="1" 
F 0 "D?" V 7500 3150 50  0000 L CNN
F 1 "PDS760" V 7400 3100 50  0000 L CNN
F 2 "RespiraWorks_Std:D_PowerDI-5" H 7400 2950 50  0001 C CNN
F 3 "~" H 7400 2950 50  0001 C CNN
F 4 "Diodes Inc." V 7400 2950 50  0001 C CNN "Manufacturer"
F 5 "PDS760-13" H 7400 2950 50  0001 C CNN "Manufacturer PN"
	1    7400 2950
	0    1    1    0   
$EndComp
Connection ~ 7400 2800
Wire Wire Line
	7400 2800 7300 2800
$Comp
L power:GND #PWR?
U 1 1 5FDAF316
P 7400 3150
AR Path="/5FCD4B8E/5FCD4BC5/5FDAF316" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDAF316" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7400 2900 50  0001 C CNN
F 1 "GND" H 7405 2977 50  0000 C CNN
F 2 "" H 7400 3150 50  0001 C CNN
F 3 "" H 7400 3150 50  0001 C CNN
	1    7400 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3150 7500 3150
Connection ~ 7400 3150
Text Notes 9200 2450 0    50   ~ 0
28.7uF @ 5V\nESR = 2-10 mΩ, 10 kHz-7 Mhz\n(Adds zero ≥500 kHz)
Wire Wire Line
	7100 3400 9700 3400
Connection ~ 8750 2800
Connection ~ 8200 2800
Wire Wire Line
	8200 2800 8750 2800
Wire Wire Line
	7800 2800 8200 2800
Connection ~ 9300 2800
Wire Wire Line
	9300 2800 9700 2800
Wire Wire Line
	8750 2800 9300 2800
Text Notes 2300 3150 0    50   ~ 0
8.1uF @ 12V each
$Comp
L Device:C C?
U 1 1 5FDE7DD7
P 3200 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FDE7DD7" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDE7DD7" Ref="C?"  Part="1" 
F 0 "C?" V 2950 2700 50  0000 L CNN
F 1 "100nF" V 3050 2650 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 3238 2550 50  0001 C CNN
F 3 "~" H 3200 2700 50  0001 C CNN
	1    3200 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FDE7DDD
P 3200 2850
AR Path="/5FCD4B8E/5FCD4BC5/5FDE7DDD" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDE7DDD" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3200 2600 50  0001 C CNN
F 1 "GND" H 3205 2677 50  0000 C CNN
F 2 "" H 3200 2850 50  0001 C CNN
F 3 "" H 3200 2850 50  0001 C CNN
	1    3200 2850
	1    0    0    -1  
$EndComp
Connection ~ 3200 2550
Wire Wire Line
	3200 2550 4000 2550
Connection ~ 4000 2550
Text Notes 3650 4250 0    50   ~ 0
3.8 ms soft-start time\n(10%-90%)
Text Notes 3750 2500 0    50   ~ 0
Turns on @ 8.1V in,\nturns off @ 7.0V in
$Comp
L Connector:TestPoint TP?
U 1 1 5FE25AB1
P 6900 3900
AR Path="/5FCD4B8E/5FCD4BC5/5FE25AB1" Ref="TP?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FE25AB1" Ref="TP?"  Part="1" 
F 0 "TP?" H 6842 3920 50  0000 R CNN
F 1 "~Power Good" H 6842 4018 50  0000 R CNN
F 2 "RespiraWorks_Std:TestPoint_Pad_2.0x2.0mm" H 7100 3900 50  0001 C CNN
F 3 "~" H 7100 3900 50  0001 C CNN
	1    6900 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	6900 3900 6900 3400
Text Notes 5350 4750 0    50   ~ 0
Zero @ 2 kHz
Wire Wire Line
	9700 2800 9700 3100
Wire Wire Line
	7100 3400 7100 3100
Text Notes 8700 4050 0    50   ~ 0
Vout = 0.8V * (Rhigh+Rlow)/Rlow
Text Notes 9800 3150 0    50   ~ 0
Rhigh
Text Notes 9800 3500 0    50   ~ 0
Rlow
Text HLabel 3850 3100 0    50   Input ~ 0
nEnable
Wire Wire Line
	3850 3100 4000 3100
Wire Wire Line
	9700 2800 10050 2800
$Comp
L Connector:TestPoint_Flag TP?
U 1 1 60F57642
P 10050 2750
AR Path="/5FCD5082/60F57642" Ref="TP?"  Part="1" 
AR Path="/60F57642" Ref="TP?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BC5/60F57642" Ref="TP?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/60F57642" Ref="TP?"  Part="1" 
F 0 "TP?" H 10310 2798 50  0000 L CNN
F 1 "TestPoint_Flag" H 10310 2753 50  0001 L CNN
F 2 "RespiraWorks_Std:TestPoint_Keystone_5019_Minature" H 10250 2750 50  0001 C CNN
F 3 "~" H 10250 2750 50  0001 C CNN
	1    10050 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 2750 10050 2800
Connection ~ 10050 2800
Wire Wire Line
	10050 2800 10350 2800
$Comp
L Device:C C?
U 1 1 6041E62D
P 2750 2700
AR Path="/601A64F8/6041E62D" Ref="C?"  Part="1" 
AR Path="/601A64F8/60567144/6041E62D" Ref="C?"  Part="1" 
AR Path="/601A64F8/60567D6E/6041E62D" Ref="C?"  Part="1" 
AR Path="/6023BF6A/60567D6E/6041E62D" Ref="C?"  Part="1" 
AR Path="/6041384A/60567D6E/6041E62D" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BC5/6041E62D" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/6041E62D" Ref="C?"  Part="1" 
F 0 "C?" H 2850 2600 50  0000 L CNN
F 1 "10uF" H 2850 2500 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1210_3225Metric" H 2788 2550 50  0001 C CNN
F 3 "~" H 2750 2700 50  0001 C CNN
F 4 "100V" H 2850 2400 50  0000 L CNN "Voltage"
F 5 "X7S" H 2750 2700 50  0001 C CNN "Dielectric"
F 6 "Murata" H 2750 2700 50  0001 C CNN "Manufacturer"
F 7 "GRM32EC72A106KE05L" H 2750 2700 50  0001 C CNN "Manufacturer PN"
	1    2750 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 2550 2300 2550
Connection ~ 2750 2550
Wire Wire Line
	2750 2550 3200 2550
$Comp
L power:GND #PWR?
U 1 1 6042457A
P 2750 2850
AR Path="/5FCD4B8E/5FCD4BC5/6042457A" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/6042457A" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2750 2600 50  0001 C CNN
F 1 "GND" H 2755 2677 50  0000 C CNN
F 2 "" H 2750 2850 50  0001 C CNN
F 3 "" H 2750 2850 50  0001 C CNN
	1    2750 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60426728
P 2300 2700
AR Path="/601A64F8/60426728" Ref="C?"  Part="1" 
AR Path="/601A64F8/60567144/60426728" Ref="C?"  Part="1" 
AR Path="/601A64F8/60567D6E/60426728" Ref="C?"  Part="1" 
AR Path="/6023BF6A/60567D6E/60426728" Ref="C?"  Part="1" 
AR Path="/6041384A/60567D6E/60426728" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FCD4BC5/60426728" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/60426728" Ref="C?"  Part="1" 
F 0 "C?" H 2400 2600 50  0000 L CNN
F 1 "10uF" H 2400 2500 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1210_3225Metric" H 2338 2550 50  0001 C CNN
F 3 "~" H 2300 2700 50  0001 C CNN
F 4 "100V" H 2400 2400 50  0000 L CNN "Voltage"
F 5 "X7S" H 2300 2700 50  0001 C CNN "Dielectric"
F 6 "Murata" H 2300 2700 50  0001 C CNN "Manufacturer"
F 7 "GRM32EC72A106KE05L" H 2300 2700 50  0001 C CNN "Manufacturer PN"
	1    2300 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6042672E
P 2300 2850
AR Path="/5FCD4B8E/5FCD4BC5/6042672E" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/6042672E" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2300 2600 50  0001 C CNN
F 1 "GND" H 2305 2677 50  0000 C CNN
F 2 "" H 2300 2850 50  0001 C CNN
F 3 "" H 2300 2850 50  0001 C CNN
	1    2300 2850
	1    0    0    -1  
$EndComp
Connection ~ 2300 2550
Wire Wire Line
	2300 2550 2750 2550
$EndSCHEMATC
