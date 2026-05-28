import socket
import sys
import argparse
import json


def main():
    parser = argparse.ArgumentParser(description='MCM subscriber client for OScar')
    parser.add_argument('--server_ip_address', nargs='?', default="127.0.0.1", help='OScar JSON server address')
    parser.add_argument('--server_port', nargs='?', default=5100, type=int, help='OScar JSON server port')
    args = parser.parse_args()

    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.connect((args.server_ip_address, args.server_port))

    welcome_msg = tcp_sock.recv(1024).decode("utf-8")
    if welcome_msg != "Connection: confirmed":
        print(f"Error: received a wrong confirmation message: {welcome_msg}")
        sys.exit(1)

    print("Connessione confermata. Invio MCM_subscribe...")

    request = {"request_type": "MCM_subscribe"}
    tcp_sock.sendall(bytes(json.dumps(request), encoding="utf-8"))

    decoder = json.JSONDecoder()
    buffer = ""

    try:
        while True:
            chunk = tcp_sock.recv(65536)
            if not chunk:
                print("Connessione chiusa dal server.")
                break

            buffer += chunk.decode("utf-8", errors="replace")
            buffer = buffer.lstrip("\x00 \t\r\n")

            while buffer:
                try:
                    obj, idx = decoder.raw_decode(buffer)
                except json.JSONDecodeError:
                    break

                print("Ricevuto JSON:")
                print(json.dumps(obj, indent=2, sort_keys=False))
                print("-" * 60)

                buffer = buffer[idx:].lstrip("\x00 \t\r\n")
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente. Invio MCM_unsubscribe...")
        try:
            tcp_sock.sendall(bytes(json.dumps({"request_type": "MCM_unsubscribe"}) + "\0", encoding="utf-8"))
        except OSError:
            pass
    finally:
        tcp_sock.close()


if __name__ == "__main__":
    main()
