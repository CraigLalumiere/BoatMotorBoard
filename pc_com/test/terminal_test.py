import pytest
from pc_com import terminal

@pytest.fixture
def console(qtbot):
    console = terminal.Terminal(None)
    qtbot.addWidget(console)
    return console

def test_terminal_clean_when_text_expect_text(console):
    console.feed(b'test')

    assert console.toPlainText() == 'test'

def test_terminal_with_text_when_cursor_left_1_expect_cursor_left_1(console):
    console.feed(b'test')
    console.feed(b'\x1b[D')  # Cursor left
    assert console.textCursor().position() == 3  

def test_terminal_with_text_when_cursor_left_3_expect_cursor_left_3(console):
    console.feed(b'test')
    console.feed(b'\x1b[3D')  # Cursor left 3
    assert console.textCursor().position() == 1  

def test_terminal_with_text_when_cursor_right_1_expect_cursor_same_pos(console):
    console.feed(b'test')
    console.feed(b'\x1b[C')  # Cursor right
    assert console.textCursor().position() == 4  # Cursor should be at the end of the text, not shifted 

def test_terminal_with_text_with_cursor_start_when_cursor_right_3_expect_cursor_pos_right_3(console):
    console.feed(b'test')
    console.cursor_position = 0 # Set cursor to start
    console.feed(b'\x1b[3C')  # Cursor right 3
    assert console.textCursor().position() == 3  # Cursor should be at the end of the text, not shifted right        

def test_terminal_with_text_when_cursor_saved_expect_cursor_pos_saved(console):
    console.feed(b'test')
    console.feed(b'\x1b[s')  # Cursor save
    assert console.saved_cursor_pos == 4

def test_terminal_with_text_when_cursor_restored_expect_cursor_pos_restored(console):
    console.feed(b'test')
    console.saved_cursor_pos = 2
    console.feed(b'\x1b[u')  # Cursor restore
    assert console.textCursor().position() == 2  

def test_terminal_with_text_when_cursor_insert_character_expect_blank_inserted(console):
    console.feed(b'test')
    console.feed(b'\x1b[@')  # Cursor insert character
    assert console.toPlainText() == 'test '  # Space should be inserted at the end
    
def test_terminal_with_text_when_cursor_delete_character_expect_text_deleted(console):
    console.feed(b'test')
    console.cursor_position = 3 # Set cursor to end t   
    console.feed(b'\x1b[P')  # Cursor delete character
    assert console.toPlainText() == 'tes'  # Last character should be deleted

def test_terminal_with_text_when_cursor_not_handled_expect_normal_state(console):
    console.feed(b'test')
    console.feed(b'\x1b[x')  # Not handled sequence
    assert console.state == terminal.Terminal.NORMAL  # Should return to normal state
    assert console.toPlainText() == 'test'  # Text should remain unchanged

def test_terminal_with_text_when_cursor_insert_character_beginning_expect_blank_inserted(console):
    console.feed(b'test')
    console.cursor_position = 0  # Set cursor to start
    console.feed(b'\x1b[@')  # Cursor insert character
    assert console.toPlainText() == ' test'  # Space should be inserted at the beginning

def test_terminal_with_text_when_cursor_delete_character_beginning_expect_text_deleted(console):
    console.feed(b'test')
    console.cursor_position = 0  # Set cursor to start
    console.feed(b'\x1b[P')  # Cursor delete character
    assert console.toPlainText() == 'est'  # First character should be deleted, est should remain

def test_terminal_with_text_when_cursor_insert_character_middle_expect_blank_inserted(console):
    console.feed(b'test')
    console.cursor_position = 2  # Set cursor to middle
    console.feed(b'\x1b[@')  # Cursor insert character
    assert console.toPlainText() == 'te st'  # Space should be inserted in the middle

def test_terminal_with_text_when_cursor_delete_character_middle_expect_text_deleted(console):
    console.feed(b'test')
    console.cursor_position = 2  # Set cursor to middle
    console.feed(b'\x1b[P')  # Cursor delete character
    assert console.toPlainText() == 'tet'  # Character at position 2 should be deleted, resulting in 'tes'