import serial
import threading
import queue

from hdlc import (HDLC, HDLCStatus)


def get_all_from_queue(q):
    try:
        while True:
            yield q.get_nowait()
    except queue.Empty:
        pass


class COMThread(threading.Thread):
    def __init__(self,
                 data_q, event_q, command_q,
                 port_num,
                 port_baud,
                 port_bytesize=serial.EIGHTBITS,
                 port_parity=serial.PARITY_NONE,
                 port_stopbits=serial.STOPBITS_ONE,
                 port_timeout=0.01,
                 port_xonxoff=0,  # software flow control
                 port_rtscts=0,   # hardware (RTS/CTS) flow control
                 port_dsrdtr=0    # hardware(DSR/DTR) flow control
                 ):

        threading.Thread.__init__(self)

        self.serial_port = None
        self.serial_arg = dict(port=port_num,
                               baudrate=port_baud,
                               bytesize=port_bytesize,
                               parity=port_parity,
                               stopbits=port_stopbits,
                               timeout=port_timeout,
                               xonxoff=port_xonxoff,
                               rtscts=port_rtscts,
                               dsrdtr=port_dsrdtr
                               )
        self.event_q = event_q
        self.data_q = data_q
        self.command_q = command_q

        self.hdlc = HDLC()

        self.alive = threading.Event()
        self.alive.set()

    def run(self):
        try:
            if self.serial_port:
                self.serial_port.close()

            self.serial_port = serial.Serial(**self.serial_arg)
            self.serial_port.reset_input_buffer()

            while self.alive.is_set():
                # send the command packets in queue
                for command in get_all_from_queue(self.command_q):
                    # frame the command packet with HDLC
                    command_framed = self.hdlc.frame_packet(command)
                    self.serial_port.write(command_framed)
                    # print("sending: {}".format(command_framed))

                # Read 1 char, blocking for "port_timeout" seconds until it's received
                raw_packet = self.serial_port.read(1)
                # If data was received (avoid null packet from timeout)
                if raw_packet:
                    # Read the rest of the data available
                    raw_packet += self.serial_port.read(self.serial_port.in_waiting)

                    # iterate through each byte received and parse
                    for b in raw_packet:
                        status = self.hdlc.add_byte(b)
                        if status == HDLCStatus.MSG_OK:
                            self.data_q.put(self.hdlc.last_message)
                            # print(self.hdlc.last_message)

            # clean up
            if self.serial_port:
                self.serial_port.close()

        except serial.SerialException as e:
            # clean up
            if self.serial_port:
                self.serial_port.close()

            self.event_q.put({'event': 'error',
                              'data': str(e)})

            return                

    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
