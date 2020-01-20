#!/bin/bash

sed -n 's/^.*\([[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]]\):\([[:xdigit:]][[:xdigit:]]\).*$/\100\2/p' text.txt > res.txt

sed -n 's/^.*\([[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]]\):\([[:xdigit:]] \).*$/\1000\2/p' text.txt >> res.txt

#cat res.txt 

sort -o res.txt res.txt 

sed -n 's/\(.*\)/#define IND_        0x\1/p' res.txt >LXM32A_CANopen_register.h




#cat res.txt
