<?php


if (empty($argv[1])) {
    echo "Usage php pattern_convert.php path/to/file\n";
    exit(1);
}

$file = $argv[1];
if (!file_exists($file)) {
    echo "$file does not exist\n";
    exit(1);
}


echo "// Converted from http://www.sxlist.com/TECHREF/datafile/charset/8x6.htm
const uint8_t font[SOURCE_SIZE_FONT][8] PROGMEM = {\n";

$lines = file($file);
foreach ($lines as $k => $line) {
    //$k = 30;
    //$line = $lines[$k];


    $line = trim($line);
    if (empty($line)) continue;


    if ($line[0] == '{') {
        $frame = array();
        preg_match('|{(.*?),(.*?),(.*?),(.*?),(.*?),(.*?)}, (.*?)$|', $line, $matches);
        array_shift($matches);
        $comment = array_pop($matches);

        foreach ($matches as $byte) {
            $bin = base_convert($byte, 16, 2);
            $frame[] = sprintf("%08d", $bin);
        }
        frame_Rotate($frame);

        // Format for arduino.
        echo '{';
        foreach ($frame as $i => $row) {
            $dec = base_convert($row, 2, 10);
            echo sprintf('0x%02X', $dec);
            if ($i < 7) echo ',';
        }
        echo '},' . $comment . "\n";

    }
}

echo "\n};\n";

function frame_Rotate(&$frame, $degrees = 270)
{
    $tmp = array();

    for ($i = 0; $i < 8; $i++){
        if (!isset($frame[$i])) {
            $frame[$i] = '00000000';
        }
        $tmp[$i] = $frame[$i];
    }

    $frame = array();

    if ($degrees == 270) {
        for ($j = 7; $j >= 0; $j--) {

            $row = '';
            for ($x = 0; $x<8; $x++) {
                $row .= $tmp[$x][$j];
            }
            $frame[] = $row;
        }
    }
}

