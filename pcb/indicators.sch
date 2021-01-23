EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Indicators"
Date ""
Rev "2.0"
Comp "RespiraWorks"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Conn_01x10_Female J?
U 1 1 600A179B
P 5100 2100
F 0 "J?" H 5127 2076 50  0000 L CNN
F 1 "Conn_01x10_Female" H 5127 1985 50  0000 L CNN
F 2 "" H 5100 2100 50  0001 C CNN
F 3 "~" H 5100 2100 50  0001 C CNN
	1    5100 2100
	1    0    0    -1  
$EndComp
Text HLabel 4300 1700 0    50   Input ~ 0
CC-LED-G
Text HLabel 4300 1800 0    50   Input ~ 0
CC-LED-R
Text HLabel 4300 1900 0    50   Input ~ 0
CC-LED-Y
Text HLabel 4300 2000 0    50   Input ~ 0
UI-LED-G
Text HLabel 4300 2100 0    50   Input ~ 0
UI-LED-R
Text HLabel 4300 2200 0    50   Input ~ 0
UI-LED-Y
Wire Wire Line
	4300 1700 4900 1700
Wire Wire Line
	4300 1800 4900 1800
Wire Wire Line
	4300 1900 4900 1900
Wire Wire Line
	4300 2000 4900 2000
Wire Wire Line
	4300 2100 4900 2100
Wire Wire Line
	4300 2200 4900 2200
$Comp
L power:+5V #PWR?
U 1 1 600A1896
P 3550 1600
F 0 "#PWR?" H 3550 1450 50  0001 C CNN
F 1 "+5V" H 3565 1773 50  0000 C CNN
F 2 "" H 3550 1600 50  0001 C CNN
F 3 "" H 3550 1600 50  0001 C CNN
	1    3550 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 600A18CF
P 3550 2700
F 0 "#PWR?" H 3550 2450 50  0001 C CNN
F 1 "GND" H 3555 2527 50  0000 C CNN
F 2 "" H 3550 2700 50  0001 C CNN
F 3 "" H 3550 2700 50  0001 C CNN
	1    3550 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 2300 3550 2300
Wire Wire Line
	3550 2300 3550 1600
Wire Wire Line
	4900 2400 3550 2400
Wire Wire Line
	3550 2400 3550 2300
Connection ~ 3550 2300
Wire Wire Line
	4900 2500 3550 2500
Wire Wire Line
	3550 2500 3550 2600
Wire Wire Line
	4900 2600 3550 2600
Connection ~ 3550 2600
Wire Wire Line
	3550 2600 3550 2700
$EndSCHEMATC
