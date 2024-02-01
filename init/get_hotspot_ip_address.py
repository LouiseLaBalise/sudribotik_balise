import socket
import fcntl
import struct


"""
Dynamically get our hotspot IPv4 address using low-level protocol.

Return hotspot ip address or localhost one.
"""
def get_ip():
    try:
        #Create a socket using IPv4 and UDP protocol (because its a low-level protocol),
        # minimum needed to do a system call with <ioctl>
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #Get ip addresss; `inet_ntoa` converts binary IP to a human-readable format
        ip_address = socket.inet_ntoa(fcntl.ioctl(
            s.fileno(), #file descriptor used to interact with our own IPv4 information
            0x8915,  # SIOCGIFADDR, operation code used to read device's address
            struct.pack('256s', "wlan0".encode('utf-8')) #transform 'wlan0' into a bin 
        )[20:24])

        return ip_address
    
    except Exception as e:
        print("Hotspot not in use.")
        return socket.gethostbyname(socket.gethostname()) #get hostname associated ip address
