#!/usr/bin/env python3
import socket

HOST = "0.0.0.0"
PORT = 5000   # тот же порт, что ты передаёшь из C++

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    print(f"UDP server listening on {HOST}:{PORT}")

    while True:
        data, addr = sock.recvfrom(65535)
        print(f"\n=== Packet from {addr} ===")
        try:
            print("Received JSON:")
            print(data.decode("utf-8"))
        except UnicodeDecodeError:
            print("Received raw bytes:", data)

if __name__ == "__main__":
    main()

