import sys
import os
import socket
import selectors
import types
import json

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
CONFIG_FILEPATH = FILE_PATH.split("_2_ROS")[0]+"init/configuration.json"    
sys.path.insert(1, FILE_PATH.split("_2_ROS")[0]) #add parent folder to python path
from init import prettify_json

# Chargement des données clients depuis un fichier JSON
def load_clients(filename):
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}

# Enregistrement des données clients dans un fichier JSON
def save_clients(filename, clients):
    with open(filename, 'w') as file:
        json.dump(clients, file)

# Création d'une nouvelle connexion sur le serveur
def accept_wrapper(sock):

    global next_client_number

    conn, addr = sock.accept()
    print(f"Accepted connection from {addr}")

    client_data = clients.get(addr[0]) # Utilisation de l'adresse IP, uniquement,  comme clé et non l'IP + le numéro de port du client
    if client_data:
        client_number = client_data['number']
    else:
        client_number = next_client_number
        clients[addr[0]] = {'number': client_number}
        next_client_number += 1
        save_clients(clients_file, clients)  # Enregistrement des clients mis à jour dans le fichier JSON

    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)

    print(f"Assigned client number {client_number} to {addr}")

# Traitement des événements de lecture et d'écriture sur une connexion client spécifique.
def service_connection(key, mask):
    sock = key.fileobj
    data = key.data

    # S'il y a des données disponibles pour être lues depuis la socket
    if mask & selectors.EVENT_READ:

        # Réception des données à partir de la socket (jusqu'à 1024 octets)
        recv_data = sock.recv(1024)
        #print(f"recv_data :{recv_data}")

        if recv_data:
            data.inb += recv_data
            #print(f"recv_data :{recv_data}")
            #print(f"data.inb :{data.inb}")
            if b"GET_POSITION" in data.inb:
                data.outb = b"Your position\n"
        else:
            print(f"Closing connection to {data.addr}\n")
            sel.unregister(sock)
            sock.close()

    # Si la socket est prête à envoyer des données au client
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            print(f"Echoing {data.outb!r} to {data.addr}")
            sent = sock.send(data.outb)
            data.outb = data.outb[sent:]
    else:
        print("No data available to be read or sent")

if __name__ == "__main__":

    host, port = sys.argv[1], int(sys.argv[2])
    clients_file = 'clients.json'  # Nom du fichier JSON
    clients = load_clients(clients_file)  # Chargement des clients existants depuis le fichier JSON
    next_client_number = len(clients) + 1  # Calcul du numéro du prochain client disponible

    sel = selectors.DefaultSelector()

    # Création d'un socket TCP en IPv4
    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Liaison de la socket à l'adresse IP et au port spécifié
    lsock.bind((host, port))

    # Configuration de la socket en mode écoute pour accepter les connexions entrantes.
    lsock.listen()
    print(f"Listening on {(host, port)}")

    # Configuration de la socket en mode non bloquant
    lsock.setblocking(False)

    # Enregistrement de la socket avec le selecteur sel pour surveiller si des données sont prêtes à être lues sur la socket
    sel.register(lsock, selectors.EVENT_READ, data=None)


    # Boucle principale
    try:
        while True:
            events = sel.select(timeout=None)
            for key, mask in events:
                if key.data is None:
                    accept_wrapper(key.fileobj)
                else:
                    service_connection(key, mask)
    except KeyboardInterrupt:
        print("Caught keyboard interrupt, exiting")
    finally:
        sel.close()


