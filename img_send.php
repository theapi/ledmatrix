<?php

$file = $argv[1];
if (!file_exists($file)) {
    echo "$file does not exist\n";
    exit(1);
}

/*
cs8 = 8 bit characters
-cstopb = one stop bit
raw = no messing with the data, like adding newlines
57600 = BAUD
*/
exec('stty -F /dev/ttyUSB0 cs8 -cstopb raw 57600');
$serial = fopen("/dev/ttyUSB0", "w+");
if( !$serial) {
    echo "Error opening serial\n";
    exit(1);
}


$contents = file_get_contents($file);

//@todo Convert the image to an rgb string.
$rgb = '12,4,16'; // the image contents converted to the required rgb string.
fwrite($serial, $rgb, strlen($rgb));


fclose($serial);
