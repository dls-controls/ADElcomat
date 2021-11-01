#!/usr/bin/env python3
import socket
import threading
import time
RECV_SIZE = 512
should_stop = False


def send_loop(sock):
    x = 0.0
    y = -100.0
    while not should_stop:
        sock.send(f"3 003 {x:.3f} {y:.3f}\r".encode())
        x += 0.1
        y += 0.1
        time.sleep(1/250.0)


def client_handler(sock, addr):
    global should_stop
    while True:
        data = sock.recv(RECV_SIZE)
        if not len(data):
            print(f"Closed {addr}")
            sock.close()
            return
        if data == b'd\n':
            print("Received d command")
            sock.send(b"8 1300 12 1 2019 300\r")
        elif data == b'A\n':
            print("Received A command")
            should_stop = False
            threading.Thread(target=send_loop, args=(sock,)).start()
        elif data == b's\n':
            print("Received s command")
            should_stop = True
        else:
            print(f"Not recognised command {repr(data)}")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 4002))
    sock.listen(5)
    while True:
        client_sock, client_addr = sock.accept()
        print(f"Got client {client_addr}")
        threading.Thread(target=client_handler,
                         args=(client_sock, client_addr)).start()
    sock.close()


if __name__ == "__main__":
    main()
