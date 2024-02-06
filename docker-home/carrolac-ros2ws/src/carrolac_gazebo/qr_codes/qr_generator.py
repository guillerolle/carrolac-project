import segno

for i in range(10):
    sign_id = "{:04d}".format(i)
    qr = segno.make_qr(sign_id)
    qr.save("qrs/" + sign_id + ".png",
            scale=10,
            border=1)
