import urllib.request
import time
import numpy as np
import socket
from pysticks import get_controller

# URL (is displayed on arduino side)
UDP_IP = "192.168.184.133"
UDP_PORT = 2390
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
con = get_controller()

def call(url):
    n = urllib.request.urlopen(url).read() # get the raw html data in bytes (sends request and warn our esp8266)
    n = n.decode("utf-8") # convert raw html bytes format to string :3

    # data = n
    data = n.split() 			#<optional> split datas we got. (if you programmed it to send more than one value) It splits them into seperate list elements.
    #data = list(map(int, data)) #<optional> turn datas to integers, now all list elements are integers.
    return data

def encode(con):
    con = np.array(con)
    # all the different commands (-1,1) are converted into a string with positive numbers (0,1) at a certain resolution
    res = 3
    con = np.round(con, res)
    con = compress(con)
    num = 0
    for i in range(len(con)):
        con_clipped = np.clip(con[i],0,1-10**-res)
        num += int(con_clipped*10**res) * 10**(i*res)
    return str(num).zfill(len(con)*res)

def compress(con):
    return con/2 + 0.5

def call(con):
    sock.sendto(bytes(str(con), "utf-8"), (UDP_IP, UDP_PORT))

# Example usage
while True:
    start = time.time()
    con.update()
    action = [con.getThrottle(), con.getRoll(), con.getPitch(), con.getYaw()]
    action_enc = encode(action)
    call(action_enc)
    print(str(np.round(1/(time.time() - start), 2)) + " Hz")