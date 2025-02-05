from pyzbar.pyzbar import decode
from PIL import Image
import qrcode

# Data to be encoded
data = 'http://10.68.0.128:1880/setPosition?x=1.0214451871186694&y=-0.5222864607813821&z=0.026685548359222468&w=0.9996438773427103'

# Generate QR code
qr = qrcode.QRCode(version=1, box_size=10, border=5)
qr.add_data(data)
qr.make(fit=True)
img = qr.make_image(fill='black', back_color='white')

# Save the image
img.save('qrcode.png')