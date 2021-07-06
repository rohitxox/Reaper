#degree to encoder
def conv(val) :
    degree  = round(val * 0.08789)
    #encoder = round(val/ 0.08789)
    #POS = encoder
    POS = degree
    return POS

print(conv(2202))

