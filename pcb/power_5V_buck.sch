EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 30
Title "5V Buck Converter"
Date ""
Rev "2.0"
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
F 1 "249K 1%" H 4650 3700 50  0000 L CNN
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
F 1 "16.9K 1%" H 5200 3900 50  0000 L CNN
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
F 1 "4.7nF 100V NP0" H 5000 4150 50  0000 L CNN
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
F 1 "47pF 100V NP0" H 6065 4205 50  0000 L CNN
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
F 1 "10nF 100V NP0" H 3750 3600 50  0000 L CNN
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
F 1 "90.9K 1%" H 3550 3200 50  0000 L CNN
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
F 1 "442K 1%" H 3600 2900 50  0000 L CNN
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
	2800 2550 3200 2550
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
F 1 "100nF 100V X7R" V 7000 2300 50  0000 L CNN
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
F 1 "52.3K 1%" H 9250 3200 50  0000 L CNN
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
F 1 "10K 1%" H 9300 3450 50  0000 L CNN
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
F 1 "7.2uH 6A" V 7749 2800 50  0000 C CNN
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
F 1 "100nF 100V X7R" H 7900 2600 50  0000 L CNN
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
F 1 "1uF 10V X7R" H 8450 2600 50  0000 L CNN
F 2 "RespiraWorks_Std:C_0603_1608Metric" H 8788 2800 50  0001 C CNN
F 3 "~" H 8750 2950 50  0001 C CNN
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
F 1 "47uF 10V X7R" H 8850 2600 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1210_3225Metric" H 9338 2800 50  0001 C CNN
F 3 "~" H 9300 2950 50  0001 C CNN
F 4 "Murata" H 9300 2950 50  0001 C CNN "Manufacturer"
F 5 "GRM32ER71A476KE15L" H 9300 2950 50  0001 C CNN "Part Number"
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
$Comp
L Device:C C?
U 1 1 5FD91006
P 2800 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FD91006" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD91006" Ref="C?"  Part="1" 
F 0 "C?" V 2550 2700 50  0000 L CNN
F 1 "2.2uF 100V X7S" V 2650 2650 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1206_3216Metric" H 2838 2550 50  0001 C CNN
F 3 "~" H 2800 2700 50  0001 C CNN
F 4 "Murata" H 2800 2700 50  0001 C CNN "Manufacturer"
F 5 "GCM31CC72A225KE02L" H 2800 2700 50  0001 C CNN "Part Number"
	1    2800 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD9100C
P 2800 2850
AR Path="/5FCD4B8E/5FCD4BC5/5FD9100C" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD9100C" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2800 2600 50  0001 C CNN
F 1 "GND" H 2805 2677 50  0000 C CNN
F 2 "" H 2800 2850 50  0001 C CNN
F 3 "" H 2800 2850 50  0001 C CNN
	1    2800 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FD92E5D
P 2350 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FD92E5D" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD92E5D" Ref="C?"  Part="1" 
F 0 "C?" V 2100 2700 50  0000 L CNN
F 1 "2.2uF 100V X7S" V 2200 2650 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1206_3216Metric" H 2388 2550 50  0001 C CNN
F 3 "~" H 2350 2700 50  0001 C CNN
F 4 "Murata" H 2350 2700 50  0001 C CNN "Manufacturer"
F 5 "GCM31CC72A225KE02L" H 2350 2700 50  0001 C CNN "Part Number"
	1    2350 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD92E63
P 2350 2850
AR Path="/5FCD4B8E/5FCD4BC5/5FD92E63" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD92E63" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 2350 2600 50  0001 C CNN
F 1 "GND" H 2355 2677 50  0000 C CNN
F 2 "" H 2350 2850 50  0001 C CNN
F 3 "" H 2350 2850 50  0001 C CNN
	1    2350 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FD937EE
P 1900 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FD937EE" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD937EE" Ref="C?"  Part="1" 
F 0 "C?" V 1650 2700 50  0000 L CNN
F 1 "2.2uF 100V X7S" V 1750 2650 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1206_3216Metric" H 1938 2550 50  0001 C CNN
F 3 "~" H 1900 2700 50  0001 C CNN
F 4 "Murata" H 1900 2700 50  0001 C CNN "Manufacturer"
F 5 "GCM31CC72A225KE02L" H 1900 2700 50  0001 C CNN "Part Number"
	1    1900 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD937F4
P 1900 2850
AR Path="/5FCD4B8E/5FCD4BC5/5FD937F4" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD937F4" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 1900 2600 50  0001 C CNN
F 1 "GND" H 1905 2677 50  0000 C CNN
F 2 "" H 1900 2850 50  0001 C CNN
F 3 "" H 1900 2850 50  0001 C CNN
	1    1900 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FD949E9
P 1400 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FD949E9" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD949E9" Ref="C?"  Part="1" 
F 0 "C?" V 1150 2700 50  0000 L CNN
F 1 "2.2uF 100V X7S" V 1250 2650 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1206_3216Metric" H 1438 2550 50  0001 C CNN
F 3 "~" H 1400 2700 50  0001 C CNN
F 4 "Murata" H 1400 2700 50  0001 C CNN "Manufacturer"
F 5 "GCM31CC72A225KE02L" H 1400 2700 50  0001 C CNN "Part Number"
	1    1400 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FD949EF
P 1400 2850
AR Path="/5FCD4B8E/5FCD4BC5/5FD949EF" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FD949EF" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 1400 2600 50  0001 C CNN
F 1 "GND" H 1405 2677 50  0000 C CNN
F 2 "" H 1400 2850 50  0001 C CNN
F 3 "" H 1400 2850 50  0001 C CNN
	1    1400 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2550 1900 2550
Connection ~ 2800 2550
Connection ~ 1900 2550
Wire Wire Line
	1900 2550 2350 2550
Connection ~ 2350 2550
Wire Wire Line
	2350 2550 2800 2550
Text HLabel 850  2550 0    50   Input ~ 0
Vin
Connection ~ 1400 2550
Text HLabel 10650 2800 2    50   Output ~ 0
Vout
$Comp
L Device:R R?
U 1 1 5FDA1BBA
P 10350 3100
AR Path="/5FCD4B8E/5FCD4BC5/5FDA1BBA" Ref="R?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDA1BBA" Ref="R?"  Part="1" 
F 0 "R?" H 10150 3150 50  0000 L CNN
F 1 "2K 1%" H 10000 3050 50  0000 L CNN
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
F 5 "PDS760-13" V 7400 2950 50  0001 C CNN "Part Number"
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
	9700 2800 10350 2800
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
Text Notes 1550 2500 0    50   ~ 0
Each 2.0uF @ 12V, 1.5uF @ 24V
$Comp
L Device:C C?
U 1 1 5FDE7DD7
P 3200 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FDE7DD7" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FDE7DD7" Ref="C?"  Part="1" 
F 0 "C?" V 2950 2700 50  0000 L CNN
F 1 "100nF 100V X7R" V 3050 2650 50  0000 L CNN
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
$Comp
L Device:C C?
U 1 1 5FE04CD9
P 950 2700
AR Path="/5FCD4B8E/5FCD4BC5/5FE04CD9" Ref="C?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FE04CD9" Ref="C?"  Part="1" 
F 0 "C?" V 700 2700 50  0000 L CNN
F 1 "2.2uF 100V X7S" V 800 2650 50  0000 L CNN
F 2 "RespiraWorks_Std:C_1206_3216Metric" H 988 2550 50  0001 C CNN
F 3 "~" H 950 2700 50  0001 C CNN
F 4 "Murata" H 950 2700 50  0001 C CNN "Manufacturer"
F 5 "GCM31CC72A225KE02L" H 950 2700 50  0001 C CNN "Part Number"
	1    950  2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE04CDF
P 950 2850
AR Path="/5FCD4B8E/5FCD4BC5/5FE04CDF" Ref="#PWR?"  Part="1" 
AR Path="/5FCD4B8E/5FE27F70/5FE04CDF" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 950 2600 50  0001 C CNN
F 1 "GND" H 955 2677 50  0000 C CNN
F 2 "" H 950 2850 50  0001 C CNN
F 3 "" H 950 2850 50  0001 C CNN
	1    950  2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  2550 950  2550
Connection ~ 950  2550
Wire Wire Line
	950  2550 1400 2550
Text Notes 3650 4250 0    50   ~ 0
3.8 ms soft-start time\n(10%-90%)
Text Notes 3750 2500 0    50   ~ 0
Turns on @ 6.5V in,\nturns off @ 5V in
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
$EndSCHEMATC