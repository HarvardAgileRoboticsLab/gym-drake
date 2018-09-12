import os

def FindResource(filename):
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), filename)

def get_full_model_path(filename):
    if filename.startswith("/"):
        full_path = filename
    else:
        full_path = os.path.join(os.path.dirname(__file__), "models", filename)
    if not os.path.exists(full_path):
        raise IOError("File %s does not exist" % full_path)
    return full_path


def Rgba2Hex(rgb):
    ''' Turn a list of R,G,B elements (any indexable
    list of >= 3 elements will work), where each element
    is specified on range [0., 1.], into the equivalent
    24-bit value 0xRRGGBB. '''
    val = 0
    for i in range(3):
        val += (256**(2 - i)) * int(255 * rgb[i])
    return val
