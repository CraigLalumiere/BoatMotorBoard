import argparse
import math
import socket
import time

from .hdlc import HDLC
from .messages.MotorData_pb2 import MotorData
from .packets import build_packet_motor_data


def make_motor_data(start_time):
    elapsed = time.monotonic() - start_time
    rpm = 2300 + 1800 * math.sin(elapsed * 0.55) + 450 * math.sin(elapsed * 2.1)
    temperature = 72 + 18 * math.sin(elapsed * 0.18) + 7 * math.sin(elapsed * 0.9)
    pressure = 38 + 8 * math.sin(elapsed * 0.4)

    msg = MotorData()
    msg.milliseconds_tick = int(elapsed * 1000)
    msg.temperature = temperature
    msg.pressure = pressure
    msg.tachometer = max(0, rpm)
    msg.vbat = 13.4 + 0.35 * math.sin(elapsed * 0.33)
    msg.engine_minutes = 1260 + int(elapsed / 60)
    msg.start = int(elapsed) % 24 < 18
    msg.neutral = int(elapsed) % 14 < 5
    msg.buzzer = int(elapsed) % 30 >= 27
    msg.temp_good = temperature < 105
    msg.pres_good = pressure > 24
    return msg


def run_server(host, port, rate_hz):
    hdlc = HDLC()
    interval = 1.0 / rate_hz

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, port))
        server.listen(1)
        print(f"MotorData simulator listening on socket://{host}:{port}")

        while True:
            conn, addr = server.accept()
            print(f"Client connected from {addr[0]}:{addr[1]}")
            start_time = time.monotonic()

            with conn:
                conn.settimeout(0.01)
                while True:
                    msg = make_motor_data(start_time)
                    frame = hdlc.frame_packet(build_packet_motor_data(msg))
                    try:
                        conn.sendall(frame)
                    except OSError:
                        print("Client disconnected")
                        break

                    try:
                        _ = conn.recv(4096)
                    except socket.timeout:
                        pass
                    except OSError:
                        print("Client disconnected")
                        break

                    time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description="Fake MotorData stream for pc_com.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    args = parser.parse_args()

    run_server(args.host, args.port, args.rate_hz)


if __name__ == "__main__":
    main()
