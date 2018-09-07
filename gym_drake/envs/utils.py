def Rgba2Hex(rgb):
    ''' Turn a list of R,G,B elements (any indexable
    list of >= 3 elements will work), where each element
    is specified on range [0., 1.], into the equivalent
    24-bit value 0xRRGGBB. '''
    val = 0
    for i in range(3):
        val += (256**(2 - i)) * int(255 * rgb[i])
return val