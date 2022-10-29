

def unpack_code_message(msg):
    # msg is of the type "X=0.10\r\nY=3.50\r\nX_next=-3.1\r\nY_next=2.0\r\nN=3\r\nL=a"

    msg_components = msg.split("\r\n")

    x = float(msg_components[0].split("=")[1])
    y = float(msg_components[1].split("=")[1])
    x_next = float(msg_components[2].split("=")[1])
    y_next = float(msg_components[3].split("=")[1])
    n = int(msg_components[4].split("=")[1])
    l = msg_components[5].split("=")[1]

    print("QR code detected:", msg_components)

    return x, y, x_next, y_next, n, l
