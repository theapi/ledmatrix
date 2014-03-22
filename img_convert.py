import Image
import sys

# Or use imagemagick
# convert -resize 8x8 -depth 3 smilie.jpg out.gif
# or for raw rgb values:
# convert -resize 8x8 -depth 3 smilie.jpg out.rgb

# http://effbot.org/imagingbook/pil-index.htm
# http://effbot.org/imagingbook/image.htm

size = 8, 8

im_orig = Image.open("in.jpg")
im = im_orig.resize(size, Image.NEAREST)
#im.convert(mode='P', dither=Image.NONE, colors=16, palette=Image.ADAPTIVE)
im.convert("P", palette=Image.ADAPTIVE, colors=256)
#im.save("out.gif", "GIF")


buf = ""
# pixels = list(im.getdata())
# pixels = im.load()
bufRed = "{"
bufGreen = "{"
bufBlue = "{"
for x in range(8):
    for y in range(8):
        red, green, blue = im.getpixel((x,y))
        # @todo http://docs.python.org/2.7/library/functions.html#bytearray
        buf += "%d,%d,%d," % (red/16, green/16, blue/16)
	#print buf
        #bufRed += "%d, " % (red /16)	
	#bufGreen += "%d, " % (green /16)	
	#bufBlue += "%d, " % (blue /16)	

        #cpixel = pixels[x, y]
        #print cpixel
    #bufRed += "},\n{"
    #bufGreen += "},\n{"
    #bufBlue += "},\n{"

print buf
#print bufRed
#print
#print bufGreen
#print
#print bufBlue
#print



