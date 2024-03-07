import socket
import fcntl
import struct


def get_optimal_ip():
    """
    Dynamically get your IPv4 address.
    If the server is connected with ethernet to the hotspot, return its ip first.
    If not, return the wifi hotspot ip address.
    If no hotspot return the localhost.

    Returns:
        - str: ip address or localhost one.
    """
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
        print("Localhost use.")
        return socket.gethostbyname(socket.gethostname()) #get hostname associated ip address


def get_ethernet_ip():
    """
    Get ethernet ipv4 address.

    Returns:
        - str : ethernet ip address or None.
    """
    try:
        #Create a socket using IPv4 and UDP protocol (because its a low-level protocol),
        # minimum needed to do a system call with <ioctl>
        socket_host = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #Connect to a google external IP address
        socket_host.connect(("8.8.8.8", 80))

        #Get socket ip address
        ip_address = socket_host.getsockname()[0]

        #Close connect
        socket_host.close()
        
        return ip_address
    
    except Exception as e:
        print(f"Error getting Ethernet IP: {e}")
        return None

if __name__=="__main__":
    print(get_ethernet_ip())