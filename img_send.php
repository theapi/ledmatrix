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
$tty = '/dev/ttyUSB1';
exec("stty -F $tty cs8 -cstopb raw 57600");
$serial = fopen($tty, "w+");
if( !$serial) {
    echo "Error opening serial\n";
    exit(1);
}

//fwrite($serial, "p2\n", 3);
//echo fread($serial, 1);

$im = new Imagick($file);
$im->resizeImage (8, 8, Imagick::FILTER_LANCZOS, 0);
//$im->writeImage('test.png');
// NB cannot use exportImagePixels as Imagick::PIXEL_CHAR is not an unsigned char as expected.
//$pixels = $im->exportImagePixels(0, 0, 8, 8, "RGB", Imagick::PIXEL_CHAR);

fwrite($serial, 'i');
for ($x=0; $x<8; ++$x) {
    for ($y=0; $y<8; ++$y) {
        $pixel = $im->getImagePixelColor($x, $y);
        $colors = $pixel->getColor();
        foreach ($colors as $k => $v) {
            if ($k != 'a') {
                echo round($v/16) . ',';
                fwrite($serial, round($v/16) . ',');
            }
        }
    }
}
$im->clear();

echo "\n";
fclose($serial);
