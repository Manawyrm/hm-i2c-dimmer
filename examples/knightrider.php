#!/bin/bash
<?php

$leds = []; 
$ledIndex = 0; 
$direction = 1;

for ($i=0; $i < 9; $i++)
{ 
	$leds[$i] = 0; 
}

while (true)
{
	$ledIndex+= $direction; 
	
	if ($ledIndex == 9 || $ledIndex == 0)
		$direction *= -1; 

	$ledIndex %= 10; 

	$leds[$ledIndex] = 255; 

	for ($led=0; $led < 9; $led++)
	{ 
		$leds[$led] -= 50;
		if ($leds[$led] < 0) $leds[$led] = 0;

		$value = $leds[$led];
		$value = (int) $value;
		`i2cset -y 2 0x32 $led $value b`;
	}
	usleep(20000);
}