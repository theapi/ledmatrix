<?php

if (empty($argv[1])) {
    $scroll = true;
} else {
    // Want to send single characters, no scrolling.
    $scroll = false;
}

if (empty($argv[2])) {
    $tty = '/dev/ttyUSB0';
} else {
    $tty = $argv[2];
}

if (empty($argv[3])) {
    $baud = '115200';
} else {
    $baud = $argv[3];
}

// Delay is only for sensing single characters.
if (empty($argv[4])) {
    $delay = 1;
} else {
    $delay = $argv[4];
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
