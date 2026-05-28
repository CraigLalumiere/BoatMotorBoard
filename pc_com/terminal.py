from PySide6 import QtWidgets, QtCore, QtGui
from PySide6.QtGui import QTextCursor

from PySide6.QtGui import QKeySequence
from PySide6.QtWidgets import QApplication

def qt_key_to_ascii(event: QtGui.QKeyEvent):
    if event.key() == QtCore.Qt.Key.Key_Return:
        return b"\r"
    elif event.key() == QtCore.Qt.Key.Key_Space:
        return b" "
    elif event.key() == QtCore.Qt.Key.Key_Enter:
        return b"\r"
    elif event.key() == QtCore.Qt.Key.Key_Tab:
        return b"\t"
    elif event.key() == QtCore.Qt.Key.Key_Backspace:
        return b"\b"
    #Not handled by the cli, it is handled in keypressevent locally
    #elif event.key() == QtCore.Qt.Key.Key_Delete:
        #return b"\x1b[P"
    elif event.key() == QtCore.Qt.Key.Key_Left and not(event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier):
        return b"\x1b[D"
    elif event.key() == QtCore.Qt.Key.Key_Right and not(event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier):
        return b"\x1b[C"
    elif event.key() == QtCore.Qt.Key.Key_Up:
        return b"\x1b[A"
    elif event.key() == QtCore.Qt.Key.Key_Down:
        return b"\x1b[B"
    elif event.text() in ('abcdefghijklmnopqrstuvwxyz'
                          'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
                          '[],=-.;/`&^~*@|#(){}$><%+?"_!'
                           "'\\"):
        return event.text().encode('utf8')
    
class Terminal(QtWidgets.QPlainTextEdit):
    key_pressed_evt = QtCore.Signal(bytes)

    # states
    (NORMAL,
     AFTER_ESC,
     AFTER_CSI_WAIT_FOR_ARG,
     AFTER_CSI_RECEIVED_ARG) = range(4)


    def __init__(self, parent=None):
        super(Terminal, self).__init__()
        self.state = Terminal.NORMAL
        self.CSI_n = None
        self.current_line_start_index = 0
        self.last_char_received = None

        self.saved_cursor_pos = None
        self.cursor_position = 0

        self.setFont(QtGui.QFont("Courier", 12))
        fmt = QtGui.QFontMetrics(self.font())
        char_width = fmt.boundingRect("w").width()
        self.setCursorWidth(char_width)

    def reset(self):
        self.state = Terminal.NORMAL
        self.CSI_n = None
        self.current_line_start_index = 0
        self.last_char_received = None

        self.saved_cursor_pos = None
        self.cursor_position = 0

        self.setPlainText("")

    def dragEnterEvent(self, e):
        # disable ability to drag text around
        return

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        # Copy (Ctrl+C)
        if event.matches(QKeySequence.Copy):
            if self.textCursor().hasSelection():
                self.copy()

        # Paste (Ctrl+V) 
        if event.matches(QKeySequence.Paste):
            text = QApplication.clipboard().text()
            if text:
                # Convert line endings to \n, handling rare case of only \r too
                text = text.replace('\r\n', '\n').replace('\r', '\n')
                for ch in text:
                    if ch == '\n':
                        self.key_pressed_evt.emit(b'\r')  
                    else:
                        self.key_pressed_evt.emit(ch.encode('utf-8', errors='ignore'))

        # Delete (Del)
        if event.key() == QtCore.Qt.Key.Key_Delete:
            cursor = self.textCursor()
            if cursor.position() < len(self.toPlainText()):
                self.key_pressed_evt.emit(b'\x1b[C')  # Move cursor right 1
                self.key_pressed_evt.emit(b"\b")  # Do a backspace

        if event.key() == QtCore.Qt.Key.Key_Home:
            # Move to the start of the current command
            start_index = self.current_line_start_index+2
            length = self.cursor_position - start_index

            for n in range(length):
                self.key_pressed_evt.emit(b'\x1b[D')

        if event.key() == QtCore.Qt.Key.Key_End:
            # Move to the start of the current command
            start_index = self.cursor_position
            length = len(self.toPlainText()) - start_index

            for n in range(length):
                self.key_pressed_evt.emit(b'\x1b[C')

        if (event.key() == QtCore.Qt.Key.Key_Left and
            event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier):
            #Need to move cursor to beginning of nearest word, before a space (cursor higlight first letter of the word)

            cursor = self.textCursor()
            text = self.toPlainText()
            current_pos = cursor.position()

            # Skip spaces at the left of the word
            pos = current_pos - 1
            while pos >= 0 and text[pos] == ' ':
                pos -= 1
            # Find the first letter of the word
            while pos >= 0 and text[pos] != ' ':
                pos -= 1

            distance = current_pos - (pos + 1)

            for i in range(distance):
                self.key_pressed_evt.emit(b'\x1b[D')
        
        if (event.key() == QtCore.Qt.Key.Key_Right and
            event.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier):
            #Need to move cursor to end of nearest word, before a space (cursor higlight space after letter)
            
            cursor = self.textCursor()
            text = self.toPlainText()
            current_pos = cursor.position()

            #Skip spaces
            pos = current_pos
            while pos < len(text) and text[pos] == ' ':
                pos += 1

            # Find the next space
            next_space = text.find(' ', pos)
            if next_space == -1:
                # No space found, move to end of text
                distance = len(text) - current_pos
            else:
                distance = next_space - current_pos

            for i in range(distance):
                self.key_pressed_evt.emit(b'\x1b[C')

        code = qt_key_to_ascii(event)
        if code is not None:
            self.key_pressed_evt.emit(code)
        
        # print(self.cursor_position)

    def mousePressEvent(self, event):
        cursor = self.textCursor()
        if cursor.selectedText():
            # restore cursor position
            cursor.setPosition(self.cursor_position)
            self.setTextCursor(cursor)

        return super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        cursor = self.textCursor()

        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            if not cursor.selectedText():
                # restore cursor position
                cursor.setPosition(self.cursor_position)
                self.setTextCursor(cursor)

        return super().mouseReleaseEvent(event)

    def feed(self, data: bytes):
        data_str = data.decode("utf-8")

        # restore cursor position (if moved by mouse)
        cursor = self.textCursor()
        cursor.setPosition(self.cursor_position)
        self.setTextCursor(cursor)

        for c in data_str:
            if self.state == Terminal.NORMAL:
                # ANSI ESC
                if c == '\x1b':
                    self.state = Terminal.AFTER_ESC
                else:
                    cursor = self.textCursor()

                    if c == '\r':
                        cursor.setPosition(self.current_line_start_index)
                        self.setTextCursor(cursor)

                    elif c == '\n' and self.last_char_received == '\r':
                        # just returned to line home, so go forward to the end of the line
                        cursor.setPosition(len(self.toPlainText()))
                        self.setTextCursor(cursor)
                        cursor.insertText(c)
                        self.current_line_start_index = cursor.position()

                    else:
                        # replace character at cursor if one exists
                        if cursor.position() < len(self.toPlainText()): 
                            cursor.deleteChar()

                        cursor.insertText(c)

            elif self.state == Terminal.AFTER_ESC:
                if c == '[':
                    self.state = Terminal.AFTER_CSI_WAIT_FOR_ARG
                # don't handle non-CSI ESC commands
                else:
                    self.state = Terminal.NORMAL

            else:
                # handle CSI n parameter, remain in CSI state
                if c in '123456789':
                    self.CSI_n = int(c)
                    #print(f"CSI_n: {self.CSI_n}")
                    self.state = Terminal.AFTER_CSI_RECEIVED_ARG
                else:
                    if self.state == Terminal.AFTER_CSI_WAIT_FOR_ARG:
                        self.CSI_n = 1

                    # cursor forward (right)
                    if c == 'C':
                        self.state = Terminal.NORMAL
                        cursor = self.textCursor()
                        for n in range(self.CSI_n):
                            cursor.movePosition(QTextCursor.MoveOperation.Right)
                        self.setTextCursor(cursor)

                    # cursor backward (left)
                    # just need to handle single moves at a time
                    elif c == 'D':
                        self.state = Terminal.NORMAL
                        cursor = self.textCursor()
                        for n in range(self.CSI_n):
                            cursor.movePosition(QTextCursor.MoveOperation.Left)
                        self.setTextCursor(cursor)

                    # cursor save position
                    elif c == 's':
                        self.state = Terminal.NORMAL
                        cursor = self.textCursor()
                        self.saved_cursor_pos = cursor.position()

                    # cursor restore position
                    elif c == 'u':
                        self.state = Terminal.NORMAL
                        if self.saved_cursor_pos is not None:
                            cursor = self.textCursor()
                            cursor.setPosition(self.saved_cursor_pos)
                            self.setTextCursor(cursor)

                    # cursor insert character (ICH) https://vt100.net/docs/vt510-rm/ICH.html#:~:text=The%20ICH%20sequence%20inserts%20Pn,the%20right%20margin%20are%20lost.
                    elif c == '@':
                        start_pos = cursor.position()
                        self.state = Terminal.NORMAL
                        cursor = self.textCursor()
                        cursor.insertText(" ")
                        cursor.setPosition(start_pos)
                        self.setTextCursor(cursor)

                    # cursor delete character (DCH)
                    elif c == 'P':
                        self.state = Terminal.NORMAL
                        cursor = self.textCursor()
                        cursor.deleteChar()

                    # not handled, return to normal
                    else:
                        self.state = Terminal.NORMAL

            # save last char
            self.last_char_received = c

        # save new cursor position (so that it can be restored if changed by mouse)
        cursor = self.textCursor()
        self.cursor_position = cursor.position()


if __name__ == "__main__":
    import sys

    # Create the Qt application and console.
    app = QtWidgets.QApplication([])
    mainwin = QtWidgets.QMainWindow()
    title = "terminal"
    mainwin.setWindowTitle(title)

    console = Terminal(mainwin)
    mainwin.setCentralWidget(console)

#    console.feed(b't\x1b[sest\x1b[u')
    #console.feed(b'test\x1b[D\x1b[P')
    # for i in range(10):
    #     console.feed(b'test\n')
    #     console.feed(b'test\n')
    console.feed(b'\x1b[slp\x1b[u\x1b[slp\x1b[u\x1b[D\x1b[s\x1b[1Clp\x1b[u')


    # Show widget and launch Qt's event loop.
    mainwin.show()
    sys.exit(app.exec())





