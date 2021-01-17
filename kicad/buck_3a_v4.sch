EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LED Buck 3A"
Date "2020-10-30"
Rev "4"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATtiny:ATtiny45-20SU U2
U 1 1 5F9BE4AE
P 8350 2650
F 0 "U2" H 7950 3350 50  0000 R CNN
F 1 "ATtiny45-20SU" H 8200 3250 50  0000 R CNN
F 2 "Package_SO:SOIJ-8_5.3x5.3mm_P1.27mm" H 8350 2650 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 8350 2650 50  0001 C CNN
F 4 "C62689" H 8350 2650 50  0001 C CNN "LCSC"
	1    8350 2650
	1    0    0    -1  
$EndComp
$Comp
L buck3a:MAX16819ATT+T U1
U 1 1 5F9BF264
P 3450 4000
F 0 "U1" H 3450 4670 50  0000 C CNN
F 1 "MAX16820ATT" H 3450 4579 50  0000 C CNN
F 2 "buck_3a_v4:SON95P300X300X80-7N" H 3450 4000 50  0001 L BNN
F 3 "" H 3450 4000 50  0001 C CNN
F 4 "C143371" H 3450 4000 50  0001 C CNN "LCSC"
	1    3450 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5F9C0AFA
P 2450 2600
F 0 "R1" V 2243 2600 50  0000 C CNN
F 1 "0R070" V 2334 2600 50  0000 C CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 2380 2600 50  0001 C CNN
F 3 "~" H 2450 2600 50  0001 C CNN
F 4 "C76251" V 2450 2600 50  0001 C CNN "LCSC"
	1    2450 2600
	0    1    1    0   
$EndComp
$Comp
L Device:L L1
U 1 1 5F9C1A34
P 4350 2600
F 0 "L1" V 4540 2600 50  0000 C CNN
F 1 "WE7447714100" V 4449 2600 50  0000 C CNN
F 2 "Inductor_SMD:L_10.4x10.4_H4.8" H 4350 2600 50  0001 C CNN
F 3 "~" H 4350 2600 50  0001 C CNN
F 4 "C132141" V 4350 2600 50  0001 C CNN "LCSC"
	1    4350 2600
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:BSC040N08NS5 Q1
U 1 1 5F9C3BB3
P 4800 3800
F 0 "Q1" H 5005 3846 50  0000 L CNN
F 1 "BSC050N03" H 5005 3755 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TDSON-8-1" H 5000 3725 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BSC040N08NS5-DS-v02_00-EN.pdf?fileId=5546d4624ad04ef9014ae3065a7e2a05" V 4800 3800 50  0001 L CNN
F 4 "C139521" H 4800 3800 50  0001 C CNN "LCSC"
	1    4800 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 5F9C7810
P 3400 2250
F 0 "D1" H 3400 2467 50  0000 C CNN
F 1 "PDS1040" H 3400 2376 50  0000 C CNN
F 2 "Diode_SMD:D_PowerDI-5" H 3400 2250 50  0001 C CNN
F 3 "~" H 3400 2250 50  0001 C CNN
F 4 "C22452" H 3400 2250 50  0001 C CNN "LCSC"
	1    3400 2250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J5
U 1 1 5F9CF8C4
P 10000 4250
F 0 "J5" H 10080 4242 50  0000 L CNN
F 1 "Conn_01x06" H 10080 4151 50  0000 L CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x06_P2.00mm_Vertical" H 10000 4250 50  0001 C CNN
F 3 "~" H 10000 4250 50  0001 C CNN
	1    10000 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 5F9D0A62
P 4350 5750
F 0 "J3" H 4430 5742 50  0000 L CNN
F 1 "Conn_01x04" H 4430 5651 50  0000 L CNN
F 2 "Connector_JST:JST_ZE_BM04B-ZESS-TBT_1x04-1MP_P1.50mm_Vertical" H 4350 5750 50  0001 C CNN
F 3 "~" H 4350 5750 50  0001 C CNN
	1    4350 5750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 5F9D2F80
P 3250 2600
F 0 "J1" H 3200 2700 50  0000 L CNN
F 1 "LED+" H 3200 2500 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D3.0mm" H 3250 2600 50  0001 C CNN
F 3 "~" H 3250 2600 50  0001 C CNN
	1    3250 2600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 5F9D49D8
P 3550 2600
F 0 "J2" H 3550 2500 50  0000 C CNN
F 1 "LED-" H 3500 2700 50  0000 C CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D3.0mm" H 3550 2600 50  0001 C CNN
F 3 "~" H 3550 2600 50  0001 C CNN
	1    3550 2600
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5F9D60F0
P 8450 4300
F 0 "J4" H 8530 4292 50  0000 L CNN
F 1 "Conn_01x02" H 8530 4201 50  0000 L CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x02_P2.00mm_Vertical" H 8450 4300 50  0001 C CNN
F 3 "~" H 8450 4300 50  0001 C CNN
	1    8450 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5F9DA62A
P 9800 2650
F 0 "R7" H 9870 2696 50  0000 L CNN
F 1 "10k0" H 9870 2605 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 9730 2650 50  0001 C CNN
F 3 "~" H 9800 2650 50  0001 C CNN
F 4 "C17414" H 9800 2650 50  0001 C CNN "LCSC"
	1    9800 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5F9DBBCD
P 8050 4100
F 0 "R6" H 8120 4146 50  0000 L CNN
F 1 "10k0" H 8120 4055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7980 4100 50  0001 C CNN
F 3 "~" H 8050 4100 50  0001 C CNN
F 4 "C17414" H 8050 4100 50  0001 C CNN "LCSC"
	1    8050 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5F9E3F28
P 6800 4050
F 0 "R3" H 6870 4096 50  0000 L CNN
F 1 "82k5" H 6870 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6730 4050 50  0001 C CNN
F 3 "~" H 6800 4050 50  0001 C CNN
F 4 "C17835" H 6800 4050 50  0001 C CNN "LCSC"
	1    6800 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5F9E4745
P 6800 4550
F 0 "R4" H 6870 4596 50  0000 L CNN
F 1 "6k81" H 6870 4505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6730 4550 50  0001 C CNN
F 3 "~" H 6800 4550 50  0001 C CNN
F 4 "C17771" H 6800 4550 50  0001 C CNN "LCSC"
	1    6800 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5F9E71F9
P 7000 2600
F 0 "C5" H 7115 2646 50  0000 L CNN
F 1 "100n" H 7115 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7038 2450 50  0001 C CNN
F 3 "~" H 7000 2600 50  0001 C CNN
F 4 "C49678" H 7000 2600 50  0001 C CNN "LCSC"
	1    7000 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5F9E938B
P 1800 2950
F 0 "C2" H 1915 2996 50  0000 L CNN
F 1 "100n" H 1915 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1838 2800 50  0001 C CNN
F 3 "~" H 1800 2950 50  0001 C CNN
F 4 "C49678" H 1800 2950 50  0001 C CNN "LCSC"
	1    1800 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5F9EC0C8
P 6600 2600
F 0 "C4" H 6715 2646 50  0000 L CNN
F 1 "22u0" H 6715 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6638 2450 50  0001 C CNN
F 3 "~" H 6600 2600 50  0001 C CNN
F 4 "C45783" H 6600 2600 50  0001 C CNN "LCSC"
	1    6600 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F9EC8A9
P 2900 2950
F 0 "C3" H 3015 2996 50  0000 L CNN
F 1 "22u0" H 3015 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2938 2800 50  0001 C CNN
F 3 "~" H 2900 2950 50  0001 C CNN
F 4 "C45783" H 2900 2950 50  0001 C CNN "LCSC"
	1    2900 2950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0101
U 1 1 5F9EF59A
P 1200 2600
F 0 "#PWR0101" H 1200 2450 50  0001 C CNN
F 1 "VCC" H 1215 2773 50  0000 C CNN
F 2 "" H 1200 2600 50  0001 C CNN
F 3 "" H 1200 2600 50  0001 C CNN
	1    1200 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2600 1350 2600
Wire Wire Line
	2600 2600 2650 2600
Wire Wire Line
	3750 2600 4200 2600
Wire Wire Line
	4500 2600 4900 2600
Wire Wire Line
	4900 2600 4900 3600
Wire Wire Line
	3250 2250 2150 2250
Wire Wire Line
	2150 2250 2150 2600
Connection ~ 2150 2600
Wire Wire Line
	2150 2600 2300 2600
Wire Wire Line
	3550 2250 4900 2250
Wire Wire Line
	4900 2250 4900 2600
Connection ~ 4900 2600
Wire Wire Line
	4150 3800 4600 3800
$Comp
L power:+5V #PWR0102
U 1 1 5FA07D6D
P 4300 3600
F 0 "#PWR0102" H 4300 3450 50  0001 C CNN
F 1 "+5V" H 4315 3773 50  0000 C CNN
F 2 "" H 4300 3600 50  0001 C CNN
F 3 "" H 4300 3600 50  0001 C CNN
	1    4300 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 3600 4300 3600
Wire Wire Line
	2900 2600 2900 2800
Connection ~ 2900 2600
Wire Wire Line
	2900 2600 3050 2600
$Comp
L power:GND #PWR0103
U 1 1 5FA20125
P 2900 3250
F 0 "#PWR0103" H 2900 3000 50  0001 C CNN
F 1 "GND" H 2905 3077 50  0000 C CNN
F 2 "" H 2900 3250 50  0001 C CNN
F 3 "" H 2900 3250 50  0001 C CNN
	1    2900 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3100 2900 3250
Wire Wire Line
	2650 2600 2650 3800
Wire Wire Line
	2650 3800 2750 3800
Connection ~ 2650 2600
Wire Wire Line
	2650 2600 2900 2600
Wire Wire Line
	2150 2600 2150 4000
Wire Wire Line
	2150 4000 2750 4000
Wire Wire Line
	2750 3900 2450 3900
Text Label 2450 3900 0    50   ~ 0
PWM
Wire Wire Line
	1350 2800 1350 2600
Connection ~ 1350 2600
Wire Wire Line
	1350 2600 1800 2600
Wire Wire Line
	1800 2800 1800 2600
Connection ~ 1800 2600
Wire Wire Line
	1800 2600 2150 2600
Wire Wire Line
	1350 3100 1350 3200
Wire Wire Line
	1350 3200 1600 3200
Wire Wire Line
	1800 3200 1800 3100
$Comp
L power:GND #PWR0104
U 1 1 5FA3235C
P 1600 3200
F 0 "#PWR0104" H 1600 2950 50  0001 C CNN
F 1 "GND" H 1605 3027 50  0000 C CNN
F 2 "" H 1600 3200 50  0001 C CNN
F 3 "" H 1600 3200 50  0001 C CNN
	1    1600 3200
	1    0    0    -1  
$EndComp
Connection ~ 1600 3200
Wire Wire Line
	1600 3200 1800 3200
$Comp
L power:GND #PWR0105
U 1 1 5FA3500D
P 4150 4400
F 0 "#PWR0105" H 4150 4150 50  0001 C CNN
F 1 "GND" H 4155 4227 50  0000 C CNN
F 2 "" H 4150 4400 50  0001 C CNN
F 3 "" H 4150 4400 50  0001 C CNN
	1    4150 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5FA35E97
P 2750 4200
F 0 "#PWR0106" H 2750 3950 50  0001 C CNN
F 1 "GND" H 2755 4027 50  0000 C CNN
F 2 "" H 2750 4200 50  0001 C CNN
F 3 "" H 2750 4200 50  0001 C CNN
	1    2750 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 4000 4900 4150
$Comp
L power:GND #PWR0107
U 1 1 5FA38D2B
P 4900 4150
F 0 "#PWR0107" H 4900 3900 50  0001 C CNN
F 1 "GND" H 4905 3977 50  0000 C CNN
F 2 "" H 4900 4150 50  0001 C CNN
F 3 "" H 4900 4150 50  0001 C CNN
	1    4900 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5F9EA3C0
P 1350 2950
F 0 "C1" H 1465 2996 50  0000 L CNN
F 1 "22u0" H 1465 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1388 2800 50  0001 C CNN
F 3 "~" H 1350 2950 50  0001 C CNN
F 4 "C45783" H 1350 2950 50  0001 C CNN "LCSC"
	1    1350 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2450 7000 2300
Wire Wire Line
	7000 2300 6800 2300
Wire Wire Line
	6600 2300 6600 2450
Wire Wire Line
	6600 2750 6600 2900
Wire Wire Line
	6600 2900 6800 2900
Wire Wire Line
	7000 2900 7000 2750
$Comp
L power:+5V #PWR0108
U 1 1 5FA4D335
P 6800 2300
F 0 "#PWR0108" H 6800 2150 50  0001 C CNN
F 1 "+5V" H 6815 2473 50  0000 C CNN
F 2 "" H 6800 2300 50  0001 C CNN
F 3 "" H 6800 2300 50  0001 C CNN
	1    6800 2300
	1    0    0    -1  
$EndComp
Connection ~ 6800 2300
Wire Wire Line
	6800 2300 6600 2300
$Comp
L power:GND #PWR0109
U 1 1 5FA4DB85
P 6800 2900
F 0 "#PWR0109" H 6800 2650 50  0001 C CNN
F 1 "GND" H 6805 2727 50  0000 C CNN
F 2 "" H 6800 2900 50  0001 C CNN
F 3 "" H 6800 2900 50  0001 C CNN
	1    6800 2900
	1    0    0    -1  
$EndComp
Connection ~ 6800 2900
Wire Wire Line
	6800 2900 7000 2900
$Comp
L power:+5V #PWR0110
U 1 1 5FA4EECD
P 8350 1900
F 0 "#PWR0110" H 8350 1750 50  0001 C CNN
F 1 "+5V" H 8365 2073 50  0000 C CNN
F 2 "" H 8350 1900 50  0001 C CNN
F 3 "" H 8350 1900 50  0001 C CNN
	1    8350 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 1900 8350 2050
Wire Wire Line
	8350 3250 8350 3400
$Comp
L power:GND #PWR0111
U 1 1 5FA50ECF
P 8350 3400
F 0 "#PWR0111" H 8350 3150 50  0001 C CNN
F 1 "GND" H 8355 3227 50  0000 C CNN
F 2 "" H 8350 3400 50  0001 C CNN
F 3 "" H 8350 3400 50  0001 C CNN
	1    8350 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2450 9400 2450
Text Label 9250 2450 0    50   ~ 0
PWM
Wire Wire Line
	9800 2850 9800 2800
Wire Wire Line
	8950 2850 9800 2850
Wire Wire Line
	9800 2500 9800 2450
$Comp
L power:+5V #PWR0112
U 1 1 5FA6259E
P 9800 2450
F 0 "#PWR0112" H 9800 2300 50  0001 C CNN
F 1 "+5V" H 9815 2623 50  0000 C CNN
F 2 "" H 9800 2450 50  0001 C CNN
F 3 "" H 9800 2450 50  0001 C CNN
	1    9800 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2650 9400 2650
Text Label 9250 2650 0    50   ~ 0
VMESS
Wire Wire Line
	8950 2750 9400 2750
Text Label 9250 2750 0    50   ~ 0
TMESS
Wire Wire Line
	8950 2550 9400 2550
Text Label 9250 2550 0    50   ~ 0
MODE
Text Label 7150 4300 0    50   ~ 0
VMESS
$Comp
L power:VCC #PWR0113
U 1 1 5FA7DC74
P 6800 3900
F 0 "#PWR0113" H 6800 3750 50  0001 C CNN
F 1 "VCC" H 6815 4073 50  0000 C CNN
F 2 "" H 6800 3900 50  0001 C CNN
F 3 "" H 6800 3900 50  0001 C CNN
	1    6800 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5FA7E09A
P 6800 4700
F 0 "#PWR0114" H 6800 4450 50  0001 C CNN
F 1 "GND" H 6805 4527 50  0000 C CNN
F 2 "" H 6800 4700 50  0001 C CNN
F 3 "" H 6800 4700 50  0001 C CNN
	1    6800 4700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0115
U 1 1 5FA9A164
P 8050 3950
F 0 "#PWR0115" H 8050 3800 50  0001 C CNN
F 1 "+5V" H 8065 4123 50  0000 C CNN
F 2 "" H 8050 3950 50  0001 C CNN
F 3 "" H 8050 3950 50  0001 C CNN
	1    8050 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4250 8050 4300
Wire Wire Line
	8050 4300 8250 4300
$Comp
L power:GND #PWR0116
U 1 1 5FA9B588
P 8250 4400
F 0 "#PWR0116" H 8250 4150 50  0001 C CNN
F 1 "GND" H 8255 4227 50  0000 C CNN
F 2 "" H 8250 4400 50  0001 C CNN
F 3 "" H 8250 4400 50  0001 C CNN
	1    8250 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4300 7650 4300
Connection ~ 8050 4300
Text Label 7900 4300 2    50   ~ 0
TMESS
$Comp
L power:+5V #PWR0117
U 1 1 5FAA8BAA
P 9800 4550
F 0 "#PWR0117" H 9800 4400 50  0001 C CNN
F 1 "+5V" V 9815 4678 50  0000 L CNN
F 2 "" H 9800 4550 50  0001 C CNN
F 3 "" H 9800 4550 50  0001 C CNN
	1    9800 4550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5FAA93DF
P 9800 4450
F 0 "#PWR0118" H 9800 4200 50  0001 C CNN
F 1 "GND" V 9805 4322 50  0000 R CNN
F 2 "" H 9800 4450 50  0001 C CNN
F 3 "" H 9800 4450 50  0001 C CNN
	1    9800 4450
	0    1    1    0   
$EndComp
Text Label 9250 2850 0    50   ~ 0
RESET
Wire Wire Line
	9800 4250 9550 4250
Wire Wire Line
	9800 4350 9550 4350
Wire Wire Line
	9800 4150 9550 4150
Wire Wire Line
	9800 4050 9550 4050
Text Label 9800 4250 2    50   ~ 0
RESET
Text Label 9800 4050 2    50   ~ 0
MODE
Text Label 9800 4150 2    50   ~ 0
PWM
Wire Wire Line
	8950 2350 9400 2350
Text Label 9250 2350 0    50   ~ 0
LED
Text Label 9800 4350 2    50   ~ 0
LED
$Comp
L Device:R R5
U 1 1 5FAEDD37
P 7900 5400
F 0 "R5" V 7693 5400 50  0000 C CNN
F 1 "4k75" V 7784 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 7830 5400 50  0001 C CNN
F 3 "~" H 7900 5400 50  0001 C CNN
F 4 "C17672" V 7900 5400 50  0001 C CNN "LCSC"
	1    7900 5400
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5FAEE879
P 8550 5400
F 0 "D2" H 8543 5145 50  0000 C CNN
F 1 "LED blau" H 8543 5236 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 8550 5400 50  0001 C CNN
F 3 "~" H 8550 5400 50  0001 C CNN
F 4 "C2293" H 8550 5400 50  0001 C CNN "LCSC"
	1    8550 5400
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0119
U 1 1 5FAEFE25
P 7750 5400
F 0 "#PWR0119" H 7750 5250 50  0001 C CNN
F 1 "+5V" V 7765 5528 50  0000 L CNN
F 2 "" H 7750 5400 50  0001 C CNN
F 3 "" H 7750 5400 50  0001 C CNN
	1    7750 5400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8050 5400 8400 5400
Wire Wire Line
	8700 5400 9000 5400
Text Label 8850 5400 0    50   ~ 0
LED
$Comp
L power:VCC #PWR0120
U 1 1 5FAFFD3E
P 4150 5650
F 0 "#PWR0120" H 4150 5500 50  0001 C CNN
F 1 "VCC" H 4165 5823 50  0000 C CNN
F 2 "" H 4150 5650 50  0001 C CNN
F 3 "" H 4150 5650 50  0001 C CNN
	1    4150 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5FB01294
P 4150 5950
F 0 "#PWR0121" H 4150 5700 50  0001 C CNN
F 1 "GND" H 4155 5777 50  0000 C CNN
F 2 "" H 4150 5950 50  0001 C CNN
F 3 "" H 4150 5950 50  0001 C CNN
	1    4150 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 5850 4150 5950
Connection ~ 4150 5950
$Comp
L Device:R R2
U 1 1 5FB07DCC
P 3750 5450
F 0 "R2" H 3820 5496 50  0000 L CNN
F 1 "10k0" H 3820 5405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3680 5450 50  0001 C CNN
F 3 "~" H 3750 5450 50  0001 C CNN
F 4 "C17414" H 3750 5450 50  0001 C CNN "LCSC"
	1    3750 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5750 3750 5600
Wire Wire Line
	3750 5750 4150 5750
$Comp
L power:+5V #PWR0122
U 1 1 5FB0A48B
P 3750 5300
F 0 "#PWR0122" H 3750 5150 50  0001 C CNN
F 1 "+5V" H 3765 5473 50  0000 C CNN
F 2 "" H 3750 5300 50  0001 C CNN
F 3 "" H 3750 5300 50  0001 C CNN
	1    3750 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5750 3450 5750
Connection ~ 3750 5750
Text Label 3450 5750 0    50   ~ 0
MODE
$Comp
L Mechanical:MountingHole H1
U 1 1 5FB66E8D
P 6450 6850
F 0 "H1" H 6550 6896 50  0000 L CNN
F 1 "M1" H 6550 6805 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6450 6850 50  0001 C CNN
F 3 "~" H 6450 6850 50  0001 C CNN
	1    6450 6850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5FB67DEE
P 6450 7250
F 0 "H2" H 6550 7296 50  0000 L CNN
F 1 "M2" H 6550 7205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6450 7250 50  0001 C CNN
F 3 "~" H 6450 7250 50  0001 C CNN
	1    6450 7250
	1    0    0    -1  
$EndComp
Connection ~ 6800 4300
Wire Wire Line
	6800 4300 6800 4400
Wire Wire Line
	6800 4200 6800 4300
Wire Wire Line
	6800 4300 7200 4300
$Comp
L Device:C C6
U 1 1 5FB6DC72
P 7200 4550
F 0 "C6" H 7315 4596 50  0000 L CNN
F 1 "4n70" H 7315 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7238 4400 50  0001 C CNN
F 3 "~" H 7200 4550 50  0001 C CNN
	1    7200 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4400 7200 4300
Connection ~ 7200 4300
Wire Wire Line
	7200 4300 7400 4300
Wire Wire Line
	7200 4700 6800 4700
Connection ~ 6800 4700
$EndSCHEMATC
