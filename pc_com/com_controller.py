import queue
import logging
from com_thread import COMThread
import serial.tools.list_ports
import packets


log = logging.getLogger(__name__)


def get_item_from_queue(q):
    try:
        item = q.get(True, 0.01)
    except queue.Empty:
        return None

    return item


def get_all_from_queue(q):
    """ Generator to yield one after the others all items
        currently in the queue Q, without any waiting.
    """
    try:
        while True:
            yield q.get_nowait()
    except queue.Empty:
        pass


def get_com_ports():
    ports = serial.tools.list_ports.comports()
    if ports is None:
        return []
    else:
        return [port.name for port in ports]


class ComController:
    def __init__(self):
        self.connected = False
        self.com_thread = None

        self.data_q = queue.Queue()
        self.event_q = queue.Queue()
        self.command_q = queue.Queue()

    def connect(self, port):
        if not self.connected:
            self.com_thread = COMThread(
                data_q=self.data_q,
                event_q=self.event_q,
                command_q=self.command_q,
                port_num=port,
                port_baud=500000)

            self.com_thread.start()

            com_error = get_item_from_queue(self.event_q)

            if com_error is not None:
                log.error(com_error)
                self.com_thread = None

            if self.com_thread is not None:
                log.info("controller connected")
                self.connected = True
                self.event_q.put({'event': 'connection_status_changed'})                

    def disconnect(self):
        if self.connected:
            self.com_thread.join(0.01)
            self.com_thread = None
            self.connected = False
            log.info("controller disconnected")
            self.event_q.put({'event': 'connection_status_changed'})   

    def get_received_messages(self):
        """
        Returns list of received messages
        """
        received_packets = list(get_all_from_queue(self.data_q))

        received_massages = [packets.get_message_from_packet(p) for p in received_packets]
        return received_massages
    
    def update_and_get_events(self):
        """
        Returns list of log controller events
        """
        evts = list(get_all_from_queue(self.event_q))

        # handle events as necessary, and pass them on
        for e in evts:
            self.handle_com_thread_event(e)

        return evts
    
    def handle_com_thread_event(self, evt):
        if evt['event'] == 'error':
            self.connected = False

    def transmit_cli_data(self, data: bytes):
        packet = packets.build_packet_cli_data(data)
        # print("Transmit CLI data: {0}".format(data))
        self.command_q.put(packet)

    def transmit_config_db_info_req(self):
        packet = packets.build_packet_config_db_info_req()
        self.command_q.put(packet)      

    def transmit_config_db_get_entry_req(self, entry_id):
        packet = packets.build_packet_config_db_get_entry_req(entry_id)
        self.command_q.put(packet)   

    def transmit_config_db_set_entry_req(self, msg_config_db_set_entry_req):
        packet = packets.build_packet_config_db_set_entry_req(msg_config_db_set_entry_req)
        self.command_q.put(packet)      

    def transmit_config_db_save_to_nvm_req(self):
        packet = packets.build_packet_config_db_save_no_nvm_req()
        self.command_q.put(packet)           

if __name__ == '__main__':
    """
    Simple test script. If running in pycharm, be sure to set configuration to emulate console
    Exit gracefully by sending any character
    """
    import sys
    import msvcrt
    import time
    from messages.LogPrint_pb2 import LogPrint


    log.setLevel('INFO')
    handler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%m-%d %H:%M:%S')
    handler.setFormatter(formatter)
    log.addHandler(handler)

    c = ComController()
    c.connect('COM5')

    done = False
    time.sleep(1)

    while not done:
        msgs = c.get_received_messages()
        for m in msgs:
            if isinstance(m, LogPrint):
                print("{0} {1}".format(m.milliseconds_tick, m.msg))

        evts = c.update_and_get_events()
        for e in evts:
            print(e)

        time.sleep(0.1)

        if msvcrt.kbhit():
            print("keyboard pressed, quitting")
            done = True

    c.disconnect()
