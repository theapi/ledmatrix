<?php


if (empty($argv[1])) {
    echo "Usage php img_send.php path/to/file [tty] [baud]\n";
    exit(1);
}

$file = $argv[1];
if (!file_exists($file)) {
    echo "$file does not exist\n";
    exit(1);
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

if (is_dir($file)) {
    $files = array();
    $d = dir($file);
    while (false !== ($entry = $d->read())) {
       //echo $d->path . '/' . $entry."\n";
       if (is_file($d->path . '/' . $entry)) {
           $files[] = $d->path . '/' . $entry;
       }
    }
    $d->close();
    sort($files);
} else {
    $files = array($file);
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

//fwrite($serial, "p2\n", 3);
//echo fread($serial, 1);

foreach($files as $file) {

    $im = new Imagick($file);
    $im->resizeImage (8, 8, Imagick::FILTER_LANCZOS, 0);
    //$im->writeImage('test.png');
    // NB cannot use exportImagePixels as Imagick::PIXEL_CHAR is not an unsigned char as expected.
    //$pixels = $im->exportImagePixels(0, 0, 8, 8, "RGB", Imagick::PIXEL_CHAR);

    fwrite($serial, 'i');
    for ($row=0; $row<8; ++$row) {
        for ($col=0; $col<8; ++$col) {
            $pixel = $im->getImagePixelColor($col, $row);
            $colors = $pixel->getColor();
            foreach ($colors as $k => $v) {
                if ($k != 'a') {
                    // Send binary bytes not strings (faster).
                    $byte = pack('C', round($v/16));
                    fwrite($serial, $byte . ',');
                    //echo round($v/16) . ',';
                    //fwrite($serial, round($v/16) . ',');
                }
            }
        }
    }
    //fwrite($serial, -1);
    $im->clear();

    echo "$file\n";
    //usleep(100000);
}
fclose($serial);
