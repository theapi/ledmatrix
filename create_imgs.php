<?php





for($x=0; $x<8; $x++) {
    for($y=0; $y<8; $y++) {

        $im = imagecreate (8, 8);
        $background_color = imagecolorallocate($im, 255, 255, 255);
        $red = imagecolorallocate($im, 0, 0, 255);

        imagesetpixel($im, $x, $y, $red);

        imagepng($im, 'test_' . $x . '_' . $y . '.png');
        imagedestroy($im);
    }

}

