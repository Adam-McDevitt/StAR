# command line arguments
# 1.) The string to encode in the qr code  -- to encode a larger string use speech marks on input
# 2.) The filename

# you need to install pyqrcode

import sys
import pyqrcode

input = str(sys.argv[1])
filename = str(sys.argv[2]) + ".png"

qr = pyqrcode.create(input)

print(filename)
qr.png(filename, scale=6)
