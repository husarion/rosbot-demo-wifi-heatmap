#!/bin/bash

mkdir -p ~/net_expose
touch ~/net_expose/wireless
while [ true ]; do
	cat /proc/net/wireless > ~/net_expose/wireless
	sleep 0.2
done

