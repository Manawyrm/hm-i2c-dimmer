#!/bin/bash
<?php

$i = 0;
while (true)
{
	$i++;

	for ($led=0; $led < 9; $led++)
	{ 
		$value = abs(sin($i / 2 + ($led / 9 * M_PI))) * 255;
		$value = 255 - (int) $value;
		`i2cset -y 2 0x32 $led $value b`;
	}
	usleep(20000);
}