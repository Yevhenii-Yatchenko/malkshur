#!/usr/bin/env python3
"""
TCP version of mock server for comparison with UDP
"""
import socket
import json

HOST = "0.0.0.0"
PORT = 5000

def handle_client(conn, addr):
    """Handle single client connection"""
    print(f"\n[CONNECTED] Client from {addr[0]}:{addr[1]}")

    packet_count = 0
    buffer = b""

    try:
        while True:
            # Receive data
            data = conn.recv(65535)
            if not data:
                print(f"[DISCONNECTED] Client {addr[0]}:{addr[1]}")
                break

            buffer += data

            # Try to parse complete JSON objects from buffer
            # (TCP is stream-based, may receive partial data)
            while True:
                try:
                    # Try to decode buffer as JSON
                    obj = json.loads(buffer.decode('utf-8'))

                    # Successfully parsed - print and clear buffer
                    packet_count += 1
                    print(f"\n=== Packet #{packet_count} from {addr} ===")
                    print(json.dumps(obj, indent=2))
                    print("="*50)

                    buffer = b""
                    break

                except json.JSONDecodeError as e:
                    # If error at position 0, we haven't received complete JSON yet
                    if e.pos == 0 or len(buffer) < 100:
                        break  # Wait for more data

                    # Try to find where one JSON ends and another begins
                    # Look for closing brace followed by opening brace
                    try:
                        # Find first complete JSON
                        decoder = json.JSONDecoder()
                        obj, idx = decoder.raw_decode(buffer.decode('utf-8'))

                        packet_count += 1
                        print(f"\n=== Packet #{packet_count} from {addr} ===")
                        print(json.dumps(obj, indent=2))
                        print("="*50)

                        # Remove processed JSON from buffer
                        buffer = buffer[idx:].lstrip()

                    except (json.JSONDecodeError, UnicodeDecodeError):
                        # Can't parse yet, wait for more data
                        break

                except UnicodeDecodeError:
                    # Not valid UTF-8 yet, wait for more data
                    break

    except Exception as e:
        print(f"[ERROR] Client {addr}: {e}")

    finally:
        conn.close()
        print(f"[INFO] Total packets from {addr}: {packet_count}")


def main():
    """Run TCP server"""
    print("="*60)
    print("TCP Mock Server for dd_shahed")
    print("="*60)
    print(f"Listening on {HOST}:{PORT}")
    print("Waiting for connections...")
    print("="*60)
    print()

    # Create TCP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Allow address reuse
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind and listen
    sock.bind((HOST, PORT))
    sock.listen(5)

    try:
        while True:
            # Accept connection
            conn, addr = sock.accept()

            # Handle client (single-threaded for simplicity)
            # For production, use threading or asyncio
            handle_client(conn, addr)

    except KeyboardInterrupt:
        print("\n\n[STOPPED] Server stopped by user")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
