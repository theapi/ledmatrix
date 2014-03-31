<?php

if (empty($argv[1])) {
    $delay = 1;
} else {
    $delay = $argv[1];
}

if (empty($argv[2])) {
    $scroll = false;
} else {
    $scroll = true;
}

if (empty($argv[3])) {
    $tty = '/dev/ttyUSB0';
} else {
    $tty = $argv[3];
}

if (empty($argv[4])) {
    $baud = '115200';
} else {
    $baud = $argv[4];
}

/*
cs8 = 8 bit characters
-cstopb = one stop bit
raw = no messing with the data, like adding newlines
57600 = BAUD
*/
exec("stty -F $tty cs8 -cstopb raw $baud");
$serial = fopen($tty, "w+");
if( !$serial) {
    echo "Error opening serial\n";
    exit(1);
}

while(1) {
    $line = readline('>');

    if ($scroll) {
        fwrite($serial, 's');
        fwrite($serial, $line);
        fwrite($serial, "\n");
        echo $line;

    } else {
        // One letter at a time
        $len = strlen($line);
        for ($i = 0; $i < $len; ++$i) {
            fwrite($serial, 'c');
            fwrite($serial, $line[$i]);
            fwrite($serial, "\n");
            echo $line[$i];
            // Allow it to be displayed for a while.
            // But don't delay on the last character.
            if ($i < ($len - 1)) {
                if ($delay > 0) {
                    sleep($delay);
                }
            }
        }
    }
    echo "\n";
}

fclose($serial);
