#!/bin/bash

mkdir -p ~/fakenet_
touch ~/fakenet_/wireless
while [ true ]; do
	cat /proc/net/wireless > ~/fakenet_/wireless
	sleep 0.2
done

