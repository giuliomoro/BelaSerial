#!/bin/bash
#run this on the PocketBeagle to enable UART 1 and 4

#UART1: P2.09(tx) and P2.11(rx)
#UART4: P2.05(rx) and P2.07(tx)
PINS="P2.05 P2.07 P2.09 P2.11"
for PIN in $PINS; do
	config-pin $PIN uart
done
