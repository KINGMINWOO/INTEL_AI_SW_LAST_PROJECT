"""Command-only client for USERxx accounts.

This script connects to the detection server, completes the authentication
handshake, and allows the operator to send simple textual commands without
streaming any video data. It is intended for USERxx IDs that act as
supervisory controllers (e.g., issuing `ripe@GO`).
"""

from __future__ import annotations

import socket
import sys
import threading
from typing import Tuple

AUTH_PROMPT = "AUTH_REQUEST"
AUTH_OK = "AUTH_OK"

CLIENT_ID = "USER01"
CLIENT_PASSWORD = "USER1234"
SERVER_ADDR: Tuple[str, int] = ("127.0.0.1", 9999)


def _recv_line(sock: socket.socket) -> str | None:
    """Receive a single newline-terminated line from the socket."""
    buffer = bytearray()
    while b"\n" not in buffer:
        chunk = sock.recv(1024)
        if not chunk:
            return None
        buffer.extend(chunk)
    try:
        return buffer.split(b"\n", 1)[0].decode("utf-8").strip()
    except UnicodeDecodeError:
        return None


def authenticate(sock: socket.socket) -> bool:
    """Perform the server authentication sequence for USERxx accounts."""
    prompt = _recv_line(sock)
    if prompt != AUTH_PROMPT:
        print("서버 인증 프롬프트를 받지 못했습니다.")
        return False

    credential = f"{CLIENT_ID}:{CLIENT_PASSWORD}\n".encode("utf-8")
    sock.sendall(credential)

    response = _recv_line(sock)
    if response == AUTH_OK:
        print("서버 인증을 통과했습니다.")
        return True

    print("서버 인증에 실패했습니다. ID/Password를 확인하세요.")
    return False


def listener(sock: socket.socket) -> None:
    """Background thread printing server responses."""
    buffer = bytearray()
    while True:
        try:
            chunk = sock.recv(1024)
            if not chunk:
                print("서버와의 연결이 종료되었습니다.")
                break
            buffer.extend(chunk)
            while b"\n" in buffer:
                raw, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    print(f"[서버] {line}")
        except OSError:
            print("수신 스레드에서 오류가 발생했습니다.")
            break


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(SERVER_ADDR)
        print(f"서버 {SERVER_ADDR[0]}:{SERVER_ADDR[1]}에 연결되었습니다.")

        if not authenticate(sock):
            sock.close()
            sys.exit(1)

        threading.Thread(target=listener, args=(sock,), daemon=True).start()

        print("명령을 입력하세요. 종료하려면 Ctrl+C 또는 빈 줄을 입력합니다.")
        while True:
            try:
                command = input("> ").strip()
            except (KeyboardInterrupt, EOFError):
                print("\n사용자 요청으로 종료합니다.")
                break

            if not command:
                print("빈 입력을 감지했습니다. 세션을 종료합니다.")
                break

            try:
                sock.sendall(command.encode("utf-8") + b"\n")
            except OSError:
                print("명령 전송 중 오류가 발생했습니다. 연결을 종료합니다.")
                break

    finally:
        sock.close()


if __name__ == "__main__":
    main()
