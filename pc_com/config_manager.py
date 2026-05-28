from PySide6.QtCore import (Qt, QRegularExpression)
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtWidgets import (QFrame, QGridLayout, QVBoxLayout, QLabel, QTextEdit, QLineEdit, QScrollArea, QToolButton)
from messages.ConfigDB_pb2 import ConfigEntryDataResp, ConfigDBInfoResp, ConfigDBSetEntryReq
from com_controller import ComController
from enum import Enum, auto
import json
from PySide6.QtGui import QIcon


class FileVersionMismatchError(Exception):
    def __init__(self, expected, actual):
        message = f"File is version {actual} but device expected version {expected}"
        super().__init__(message)

class ConfigManagerState(Enum):
    UNINITIALIZED = auto()
    IDLE = auto()
    READING_DATABASE = auto()


class ConfigManager:
    def __init__(self, com_controller: ComController, txt_log: QTextEdit):
        self.com_controller = com_controller
        self.txt_log = txt_log
        self.number_of_elements = None
        self.config_version = None
        self.config_entries = None

        # frame and grid set/created with register function
        self.config_view_frame = None
        self.grid = None

        self.state = ConfigManagerState.UNINITIALIZED

        # tmp
        # self.load_fake_entries()

    def load_config_entries_from_json(self, jsonFile):
        with open(jsonFile, 'r') as f:
            file_data = json.load(f)
    
            # check if file version matches firmware
            file_version = file_data['version']
        
            if file_version == self.config_version:
                entries = file_data.get('entries', [])

                for id, entry in enumerate(entries):
                    self.transmit_entry_value_change_request(id, entry['value'])
            else:
                raise FileVersionMismatchError(expected=self.config_version, actual=file_version)           
        
    def save_config_entries_to_json(self, jsonFile): 
        filtered_entries = [
            {"name": entry["name"], "value": entry["value"]}
            for entry in self.config_entries
        ]

        data_to_save = {
            "version": self.config_version,  
            "entries": filtered_entries
        }
        with open(jsonFile, 'w') as f:
            json.dump(data_to_save, f, indent=4) 

    def check_for_uncommitted_ui_changes(self):
        uncommited_changes = False

        for entry in self.config_entries:
            current_text = entry['ui_txt_value'].text()
            stored_value = entry['value']
            
            if entry['value_type'] == 'value_bool':
                stored_value = (int(stored_value))

            if current_text != str(stored_value):
                uncommited_changes = True

        return uncommited_changes
            
    def register_config_view_frame(self, frame):
        self.config_view_frame = frame 
        self.grid = QGridLayout(self.config_view_frame)
        self.grid.setAlignment(Qt.AlignTop) 


    def remove_config_entry_widgets(self):
        # no need to clear the dict entries themselves,
        # the database will be rebuild with initialize_config_entries()
        if self.config_entries is not None:
            for entry in self.config_entries:
                self.grid.removeWidget(entry['ui_label'])
                self.grid.removeWidget(entry['ui_txt_value'])
                entry['ui_label'].deleteLater()
                entry['ui_txt_value'].deleteLater()
   
    def initialize_config_entries(self):        
        # create array of blank dictionaries, to be updated later
        # runs once database info is read and we know how many entries exist
        self.config_entries = [{'name': None,
                                'value_type': None,
                                'value': None,
                                'default_value': None,
                                'ui_label': None,
                                'ui_txt_value': None
                                } 
                                for i in range(self.number_of_elements)
                               ]
        
    def load_fake_entries(self):
        self.remove_config_entry_widgets()   
        
        self.number_of_elements = 100
        self.initialize_config_entries()        

        for i in range(self.number_of_elements):
            name = "config variable {0} name".format(i)
            value_type = 'value_int32'
            value = i
            default_value = i*10

            self.create_entry(i, name, value_type, value, default_value) 

        
    def load(self):
        if self.state in [ConfigManagerState.UNINITIALIZED, ConfigManagerState.IDLE]:
            self.remove_config_entry_widgets()            
            self.number_of_elements = None
            self.config_version = None
            self.config_entries = None

            self.com_controller.transmit_config_db_info_req() 
            self.state = ConfigManagerState.READING_DATABASE

    def create_entry(self, entry_id, entry_name, entry_value_type, entry_value, entry_default_value):
        try:
            self.config_entries[entry_id]['name'] = entry_name
            self.config_entries[entry_id]['value_type'] = entry_value_type
            self.config_entries[entry_id]['value'] = entry_value
            self.config_entries[entry_id]['default_value'] = entry_default_value

            lbl_name = QLabel(entry_name + ":")
            txt_value = QLineEdit()
            txt_value.setAlignment(Qt.AlignRight)            

            match entry_value_type:
                case 'value_bool':
                    txt_value.setText(str(int(entry_value)))
                   # allow only 0 and 1
                    regex = QRegularExpression(r"[01]$")
                case 'value_uint32':
                    txt_value.setText(str(entry_value))
                    # allow only digits
                    regex = QRegularExpression(r"^\d+$")
                case 'value_int32':
                    txt_value.setText(str(entry_value))
                    # allow negative number and digits
                    regex = QRegularExpression(r"^-?\d*$")
                case 'value_float32':
                    txt_value.setText(str(entry_value))
                    # only allow negative, digits and a single period
                    regex = QRegularExpression(r"^-?\d*\.?\d*$")
                case _:
                    pass 

            validator = QRegularExpressionValidator(regex, txt_value)
            txt_value.setValidator(validator)
            
            #Revert to previously committed button
            revert_bttn = QToolButton()
            revert_bttn.setIcon(QIcon("icons/revert_icon.png"))
            revert_bttn.setToolTip("revert to previous value")
            revert_bttn.clicked.connect(lambda _, eid=entry_id: self.on_revert_clicked(eid))
            
            #Revert to default value button
            default_bttn = QToolButton()
            default_bttn.setIcon(QIcon("icons/default_icon.png"))
            default_bttn.setToolTip("revert to default value")
            default_bttn.clicked.connect(lambda _, eid=entry_id: self.on_default_clicked(eid))


            txt_value.returnPressed.connect(lambda eid=entry_id: self.on_txt_value_return_pressed(eid))
            txt_value.textChanged.connect(lambda text, eid=entry_id: self.on_txt_value_changed(eid, text))

            self.grid.addWidget(lbl_name, entry_id, 0)
            self.grid.addWidget(txt_value, entry_id, 1)
            self.grid.addWidget(revert_bttn, entry_id, 2)
            self.grid.addWidget(default_bttn, entry_id, 3)

            self.config_entries[entry_id]['ui_label'] = lbl_name
            self.config_entries[entry_id]['ui_txt_value'] = txt_value

        except IndexError as e:
            raise IndexError("config entry id out of range") from e
        
    def on_revert_clicked(self, entry_id):
        last_value = self.config_entries[entry_id]['value']
        self.config_entries[entry_id]['ui_txt_value'].setText(str(last_value))
        print(f"[Revert] Entry {entry_id} reverted to {last_value}")
        self.config_entries[entry_id]['ui_txt_value'].setStyleSheet("")

    def on_default_clicked(self, entry_id):
        default_value = self.config_entries[entry_id]['default_value']
        self.config_entries[entry_id]['ui_txt_value'].setText(str(default_value))
        self.config_entries[entry_id]['value'] = default_value 
        print(f"[Default] Entry {entry_id} reset to {default_value}")
        self.config_entries[entry_id]['ui_txt_value'].setStyleSheet("")

    def update_entry(self, entry_id, entry_name, entry_value_type, entry_value, entry_default_value):
        try:
            self.config_entries[entry_id]['name'] = entry_name
            self.config_entries[entry_id]['value_type'] = entry_value_type
            self.config_entries[entry_id]['value'] = entry_value            
            self.config_entries[entry_id]['default_value'] = entry_default_value   

            self.config_entries[entry_id]['ui_label'].setText(str(entry_name))

            if (entry_value_type == 'value_bool'):                
                self.config_entries[entry_id]['ui_txt_value'].setText(str(int(entry_value)))
            else:
                self.config_entries[entry_id]['ui_txt_value'].setText(str(entry_value)) 

            self.config_entries[entry_id]['ui_txt_value'].setStyleSheet("")                        

        except IndexError as e:
            raise IndexError("config entry id out of range") from e    

    def transmit_entry_value_change_request(self, entry_id, value):
        # build req message
        msg_config_db_set_entry_req = ConfigDBSetEntryReq()
        msg_config_db_set_entry_req.entry_id = entry_id

        # for now, convert the value into the correct type, could also raise error
        match self.config_entries[entry_id]['value_type']:
            case 'value_bool':
                msg_config_db_set_entry_req.value.value_bool = bool(int(value))
            case 'value_uint32':
                msg_config_db_set_entry_req.value.value_uint32 = int(value)
            case 'value_int32':
                msg_config_db_set_entry_req.value.value_int32 = int(value)
            case 'value_float32':
                msg_config_db_set_entry_req.value.value_float32 = float(value)
            case _:
                pass

        self.com_controller.transmit_config_db_set_entry_req(msg_config_db_set_entry_req)

    def commit_database_to_flash(self):
        self.com_controller.transmit_config_db_save_to_nvm_req() 

    def on_txt_value_changed(self, entry_id, text):
        ui_txt_value = self.config_entries[entry_id]['ui_txt_value']

        if text == str(self.config_entries[entry_id]['value']):
            ui_txt_value.setStyleSheet("")
        else:
            ui_txt_value.setStyleSheet("background-color: lightyellow;")   

    def on_txt_value_return_pressed(self, entry_id):
        ui_txt_value = self.config_entries[entry_id]['ui_txt_value']

        self.transmit_entry_value_change_request(entry_id, ui_txt_value.text())

    def handle_msg_received(self, msg):
        if isinstance(msg, ConfigDBInfoResp):
            if self.state == ConfigManagerState.READING_DATABASE:
                self.number_of_elements = msg.num_elements
                self.config_version = msg.version
                self.initialize_config_entries()

                msg_string = "db info received. length = {0}. version = {1}".format(msg.num_elements, msg.version)
                self.txt_log.append(msg_string)

                # request first entry
                if self.number_of_elements > 0:
                    self.com_controller.transmit_config_db_get_entry_req(0)
                else:
                    pass # no database exists yet!

        if isinstance(msg, ConfigEntryDataResp):
            entry_id = msg.entry_id
            entry_name = msg.name
            entry_value_type = msg.value.WhichOneof('value')
            entry_value = getattr(msg.value, entry_value_type)
            entry_default_value = getattr(msg.default_value, entry_value_type) 

            if self.state == ConfigManagerState.UNINITIALIZED:
                pass # "received config entry before database info received

            if self.state == ConfigManagerState.IDLE:
                self.update_entry(entry_id, entry_name, entry_value_type, entry_value, entry_default_value) 

            if self.state == ConfigManagerState.READING_DATABASE:
                self.create_entry(entry_id, entry_name, entry_value_type, entry_value, entry_default_value)            

                # check to see if this is the last ID to be received
                if msg.entry_id == self.number_of_elements - 1:
                    # all done
                    # TODO: enable controls
                    self.state = ConfigManagerState.IDLE
                else:
                    self.com_controller.transmit_config_db_get_entry_req(msg.entry_id + 1)


