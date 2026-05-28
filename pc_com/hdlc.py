from enum import Enum


class HDLCStatus(Enum):
    WAIT_MSG = 1   # init state. waiting for flag character to begin first msg
    IN_MSG = 2     # in the process of receiving msg
    MSG_OK = 3     # finished msg.


class HDLC(object):
    """
    This frames and unpacks framed data in the HDLC format.
    """

    # internal state
    (WAIT_FLAG, AFTER_FLAG, IN_MSG, AFTER_DLE) = range(4)

    def __init__(self, flag=0x7E, dle=0x7D, xorchr=0x20):
        self.flag = flag
        self.dle = dle
        self.xorchr = xorchr
        
        self.state = self.WAIT_FLAG
        self.last_message = bytes()
        self.message_buf = bytes()

    def frame_packet(self, packet):
        """
        Frames packet into HDLC frame.
        """
        framed = bytes([self.flag])
        for b in packet:
            if b in (self.flag, self.dle):
                framed += bytes([self.dle, b ^ self.xorchr])
            else:
                framed += bytes([b])
        framed += bytes([self.flag])
        return framed
    
    def add_byte(self, new_byte):
        """
        Unpacks HDLC frame one byte at a time.
        """    
        # waiting for first flag in stream (not aligned yet...)
        if self.state == self.WAIT_FLAG:    
            # first flag received
            if new_byte == self.flag:
                self.state = self.AFTER_FLAG                    
                return HDLCStatus.IN_MSG  
                
            # else data is still out of alignment
            else:
                return HDLCStatus.WAIT_MSG
                
        # data immediately follows a flag        
        elif self.state == self.AFTER_FLAG:
            # if a second flag occurs, re-start message
            if new_byte == self.flag:
                self.state = self.AFTER_FLAG
                return HDLCStatus.IN_MSG
            
            # an escaped message byte follows
            elif new_byte == self.dle:
                self.state = self.AFTER_DLE
                return HDLCStatus.IN_MSG
                
            # a regular message byte
            else:
                self.state = self.IN_MSG
                self.message_buf += bytes([new_byte])
                return HDLCStatus.IN_MSG                
                
        elif self.state == self.IN_MSG:
            # if currently in msg, and flag occurs, finish message
            if new_byte == self.flag:
                self.state = self.AFTER_FLAG
                self.last_message = self.message_buf
                self.message_buf = bytes()
                return HDLCStatus.MSG_OK
                
            # an escaped message byte follows
            elif new_byte == self.dle:
                self.state = self.AFTER_DLE
                return HDLCStatus.IN_MSG
                
            # a regular message byte
            else:
                self.state = self.IN_MSG
                self.message_buf += bytes([new_byte])
                return HDLCStatus.IN_MSG
                
        # an escaped message byte      
        elif self.state == self.AFTER_DLE: 
            self.state = self.IN_MSG
            self.message_buf += bytes([new_byte ^ self.xorchr])
            return HDLCStatus.IN_MSG
            
        else:
            raise AssertionError()


if __name__ == '__main__':
    pass
