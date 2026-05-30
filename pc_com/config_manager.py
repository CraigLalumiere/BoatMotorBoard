from dataclasses import dataclass
from enum import Enum, auto
from importlib import resources
import json

from PySide6.QtCore import (
    QAbstractTableModel,
    QEvent,
    QModelIndex,
    QObject,
    QRect,
    QRegularExpression,
    QSize,
    Qt,
    Signal,
)
from PySide6.QtGui import QBrush, QColor, QFont, QIcon, QRegularExpressionValidator
from PySide6.QtWidgets import QLineEdit, QStyle, QStyledItemDelegate

from .com_controller import ComController
from .messages.ConfigDB_pb2 import ConfigDBInfoResp, ConfigDBSetEntryReq, ConfigEntryDataResp


class FileVersionMismatchError(Exception):
    def __init__(self, expected, actual):
        message = f"File is version {actual} but device expected version {expected}"
        super().__init__(message)


class ConfigManagerState(Enum):
    UNINITIALIZED = auto()
    IDLE = auto()
    READING_DATABASE = auto()


@dataclass
class ConfigEntry:
    entry_id: int
    name: str
    value_type: str
    device_value: object
    editor_value: object
    default_value: object

    @property
    def is_dirty(self):
        return values_differ(self.value_type, self.editor_value, self.device_value)


TYPE_LABELS = {
    "value_bool": "Bool",
    "value_uint32": "U32",
    "value_int32": "I32",
    "value_float32": "Float",
}


def normalize_value(value_type, value):
    if value_type == "value_bool":
        if isinstance(value, str):
            return value.strip().lower() in ["1", "true", "yes", "on"]
        return bool(value)

    if value_type == "value_uint32":
        parsed = int(value)
        if parsed < 0:
            raise ValueError("Unsigned values cannot be negative")
        return parsed

    if value_type == "value_int32":
        return int(value)

    if value_type == "value_float32":
        return float(value)

    return value


def values_differ(value_type, first, second):
    if value_type == "value_float32":
        return abs(float(first) - float(second)) > 0.000001

    return normalize_value(value_type, first) != normalize_value(value_type, second)


def format_value(value_type, value):
    if value_type == "value_bool":
        return "On" if normalize_value(value_type, value) else "Off"

    if value_type == "value_float32":
        return f"{float(value):.6g}"

    return str(value)


class ConfigManager(QObject):
    database_reset = Signal()
    entry_updated = Signal(int)
    dirty_count_changed = Signal(int)
    load_progress_changed = Signal(int, int)
    state_changed = Signal(str)
    status_message = Signal(str)

    def __init__(self, com_controller: ComController, txt_log=None):
        super().__init__()
        self.com_controller = com_controller
        self.txt_log = txt_log
        self.number_of_elements = None
        self.config_version = None
        self.entries = []
        self.state = ConfigManagerState.UNINITIALIZED

    @property
    def config_entries(self):
        return self.entries

    def _set_state(self, state):
        if self.state != state:
            self.state = state
            self.state_changed.emit(state.name.replace("_", " ").title())

    def _emit_status(self, message):
        if self.txt_log is not None:
            self.txt_log.append(message)
        self.status_message.emit(message)

    def dirty_count(self):
        return sum(1 for entry in self.entries if entry is not None and entry.is_dirty)

    def _emit_dirty_count(self):
        self.dirty_count_changed.emit(self.dirty_count())

    def load_config_entries_from_json(self, json_file):
        with open(json_file, "r") as f:
            file_data = json.load(f)

        file_version = file_data["version"]
        if file_version != self.config_version:
            raise FileVersionMismatchError(expected=self.config_version, actual=file_version)

        entries = file_data.get("entries", [])
        changed = 0
        for entry_id, entry_data in enumerate(entries):
            if entry_id >= len(self.entries):
                continue
            self.set_entry_editor_value(entry_id, entry_data["value"])
            changed += 1

        self._emit_status(f"Imported {changed} config values. Review, then apply changes.")

    def save_config_entries_to_json(self, json_file):
        filtered_entries = [
            {"name": entry.name, "value": normalize_value(entry.value_type, entry.device_value)}
            for entry in self.entries
            if entry is not None
        ]

        data_to_save = {
            "version": self.config_version,
            "entries": filtered_entries,
        }
        with open(json_file, "w") as f:
            json.dump(data_to_save, f, indent=4)

        self._emit_status(f"Exported {len(filtered_entries)} config values.")

    def check_for_uncommitted_ui_changes(self):
        return self.dirty_count() > 0

    def register_config_view_frame(self, frame):
        _ = frame

    def remove_config_entry_widgets(self):
        self.entries = []
        self.database_reset.emit()
        self._emit_dirty_count()

    def initialize_config_entries(self):
        self.entries = [None for _ in range(self.number_of_elements)]
        self.database_reset.emit()
        self._emit_dirty_count()

    def load(self):
        if self.state in [ConfigManagerState.UNINITIALIZED, ConfigManagerState.IDLE]:
            self.number_of_elements = None
            self.config_version = None
            self.entries = []
            self.database_reset.emit()
            self._emit_dirty_count()
            self.com_controller.transmit_config_db_info_req()
            self._set_state(ConfigManagerState.READING_DATABASE)
            self._emit_status("Reading config database...")

    def create_entry(self, entry_id, entry_name, entry_value_type, entry_value, entry_default_value):
        entry = ConfigEntry(
            entry_id=entry_id,
            name=entry_name,
            value_type=entry_value_type,
            device_value=normalize_value(entry_value_type, entry_value),
            editor_value=normalize_value(entry_value_type, entry_value),
            default_value=normalize_value(entry_value_type, entry_default_value),
        )
        self.entries[entry_id] = entry
        self.entry_updated.emit(entry_id)
        self._emit_dirty_count()

    def update_entry(self, entry_id, entry_name, entry_value_type, entry_value, entry_default_value):
        entry = self.entries[entry_id]
        new_device_value = normalize_value(entry_value_type, entry_value)

        entry.name = entry_name
        entry.value_type = entry_value_type
        entry.default_value = normalize_value(entry_value_type, entry_default_value)
        entry.device_value = new_device_value
        entry.editor_value = new_device_value

        self.entry_updated.emit(entry_id)
        self._emit_dirty_count()

    def set_entry_editor_value(self, entry_id, value):
        entry = self.entries[entry_id]
        entry.editor_value = normalize_value(entry.value_type, value)
        self.entry_updated.emit(entry_id)
        self._emit_dirty_count()

    def revert_entry(self, entry_id):
        entry = self.entries[entry_id]
        entry.editor_value = entry.device_value
        self.entry_updated.emit(entry_id)
        self._emit_dirty_count()

    def restore_entry_default(self, entry_id):
        entry = self.entries[entry_id]
        entry.editor_value = entry.default_value
        self.entry_updated.emit(entry_id)
        self._emit_dirty_count()

    def restore_all_defaults(self):
        for entry in self.entries:
            if entry is not None:
                entry.editor_value = entry.default_value
                self.entry_updated.emit(entry.entry_id)
        self._emit_dirty_count()

    def transmit_entry_value_change_request(self, entry_id, value=None):
        entry = self.entries[entry_id]
        value_to_send = entry.editor_value if value is None else normalize_value(entry.value_type, value)

        msg_config_db_set_entry_req = ConfigDBSetEntryReq()
        msg_config_db_set_entry_req.entry_id = entry_id

        match entry.value_type:
            case "value_bool":
                msg_config_db_set_entry_req.value.value_bool = bool(value_to_send)
            case "value_uint32":
                msg_config_db_set_entry_req.value.value_uint32 = int(value_to_send)
            case "value_int32":
                msg_config_db_set_entry_req.value.value_int32 = int(value_to_send)
            case "value_float32":
                msg_config_db_set_entry_req.value.value_float32 = float(value_to_send)

        self.com_controller.transmit_config_db_set_entry_req(msg_config_db_set_entry_req)

    def apply_changed_entries(self):
        changed_entries = [entry for entry in self.entries if entry is not None and entry.is_dirty]
        for entry in changed_entries:
            self.transmit_entry_value_change_request(entry.entry_id)

        if changed_entries:
            self._emit_status(f"Applying {len(changed_entries)} config change(s) to device RAM...")
        else:
            self._emit_status("No config changes to apply.")

    def commit_database_to_flash(self):
        if self.dirty_count() > 0:
            self._emit_status("Apply edited values before saving to NVM.")
            return False

        self.com_controller.transmit_config_db_save_to_nvm_req()
        self._emit_status("Save-to-NVM request sent.")
        return True

    def handle_msg_received(self, msg):
        if isinstance(msg, ConfigDBInfoResp):
            if self.state == ConfigManagerState.READING_DATABASE:
                self.number_of_elements = msg.num_elements
                self.config_version = msg.version
                self.initialize_config_entries()

                self._emit_status(
                    f"Config database found: {msg.num_elements} entries, version {msg.version}."
                )

                if self.number_of_elements > 0:
                    self.load_progress_changed.emit(0, self.number_of_elements)
                    self.com_controller.transmit_config_db_get_entry_req(0)
                else:
                    self._set_state(ConfigManagerState.IDLE)
            return

        if isinstance(msg, ConfigEntryDataResp):
            entry_id = msg.entry_id
            entry_name = msg.name
            entry_value_type = msg.value.WhichOneof("value")
            entry_value = getattr(msg.value, entry_value_type)
            entry_default_value = getattr(msg.default_value, entry_value_type)

            if self.state == ConfigManagerState.UNINITIALIZED:
                return

            if self.state == ConfigManagerState.IDLE:
                self.update_entry(entry_id, entry_name, entry_value_type, entry_value, entry_default_value)
                return

            if self.state == ConfigManagerState.READING_DATABASE:
                self.create_entry(
                    entry_id, entry_name, entry_value_type, entry_value, entry_default_value
                )

                current = entry_id + 1
                self.load_progress_changed.emit(current, self.number_of_elements)

                if entry_id == self.number_of_elements - 1:
                    self._set_state(ConfigManagerState.IDLE)
                    self._emit_status("Config database loaded.")
                else:
                    self.com_controller.transmit_config_db_get_entry_req(entry_id + 1)


class ConfigTableModel(QAbstractTableModel):
    HEADERS = ["Name", "Value", "Default", "Type", "State", "", ""]
    COL_NAME = 0
    COL_VALUE = 1
    COL_DEFAULT = 2
    COL_TYPE = 3
    COL_STATE = 4
    COL_REVERT = 5
    COL_RESTORE_DEFAULT = 6

    def __init__(self, manager: ConfigManager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.manager.database_reset.connect(self._on_database_reset)
        self.manager.entry_updated.connect(self._on_entry_updated)

    def rowCount(self, parent=QModelIndex()):
        if parent.isValid():
            return 0
        return len(self.manager.entries)

    def columnCount(self, parent=QModelIndex()):
        if parent.isValid():
            return 0
        return len(self.HEADERS)

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None

        entry = self.manager.entries[index.row()]
        if entry is None:
            return None

        column = index.column()

        if role in [Qt.DisplayRole, Qt.EditRole]:
            if column == self.COL_NAME:
                return entry.name
            if column == self.COL_VALUE:
                return format_value(entry.value_type, entry.editor_value)
            if column == self.COL_DEFAULT:
                return format_value(entry.value_type, entry.default_value)
            if column == self.COL_TYPE:
                return TYPE_LABELS.get(entry.value_type, entry.value_type)
            if column == self.COL_STATE:
                return "Edited" if entry.is_dirty else "Applied"
            if column in [self.COL_REVERT, self.COL_RESTORE_DEFAULT]:
                return ""

        if role == Qt.ToolTipRole:
            if column == self.COL_REVERT:
                return "Revert to the currently applied value"
            if column == self.COL_RESTORE_DEFAULT:
                return "Restore the default value"

        if role == Qt.CheckStateRole and column == self.COL_VALUE and entry.value_type == "value_bool":
            return Qt.Checked if bool(entry.editor_value) else Qt.Unchecked

        if role == Qt.TextAlignmentRole:
            if column in [self.COL_REVERT, self.COL_RESTORE_DEFAULT]:
                return Qt.AlignCenter
            if column in [self.COL_VALUE, self.COL_DEFAULT]:
                return Qt.AlignRight | Qt.AlignVCenter
            return Qt.AlignLeft | Qt.AlignVCenter

        if role == Qt.BackgroundRole and entry.is_dirty:
            return QBrush(QColor("#fff4c2"))

        if role == Qt.ForegroundRole and column == self.COL_STATE:
            return QBrush(QColor("#9a6700" if entry.is_dirty else "#1f7a3f"))

        if role == Qt.FontRole and column in [self.COL_NAME, self.COL_STATE]:
            font = QFont()
            font.setBold(column == self.COL_STATE or entry.is_dirty)
            return font

        return None

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags

        flags = Qt.ItemIsSelectable | Qt.ItemIsEnabled
        entry = self.manager.entries[index.row()]
        if entry is not None and index.column() == self.COL_VALUE:
            flags |= Qt.ItemIsEditable
            if entry.value_type == "value_bool":
                flags |= Qt.ItemIsUserCheckable
        return flags

    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid() or index.column() != self.COL_VALUE:
            return False

        entry = self.manager.entries[index.row()]
        if entry is None:
            return False

        try:
            if role == Qt.CheckStateRole and entry.value_type == "value_bool":
                self.manager.set_entry_editor_value(index.row(), value == Qt.Checked)
                return True

            if role == Qt.EditRole:
                self.manager.set_entry_editor_value(index.row(), value)
                return True
        except (TypeError, ValueError):
            return False

        return False

    def _on_database_reset(self):
        self.beginResetModel()
        self.endResetModel()

    def _on_entry_updated(self, entry_id):
        top_left = self.index(entry_id, 0)
        bottom_right = self.index(entry_id, self.columnCount() - 1)
        self.dataChanged.emit(top_left, bottom_right, [])


class ConfigValueDelegate(QStyledItemDelegate):
    def __init__(self, parent=None):
        super().__init__(parent)
        icon_dir = resources.files("pc_com").joinpath("icons")
        self.revert_icon = QIcon(str(icon_dir.joinpath("revert_icon.png")))
        self.default_icon = QIcon(str(icon_dir.joinpath("default_icon.png")))

    def createEditor(self, parent, option, index):
        if index.column() != ConfigTableModel.COL_VALUE:
            return super().createEditor(parent, option, index)

        source_index = index.model().mapToSource(index) if hasattr(index.model(), "mapToSource") else index
        entry = source_index.model().manager.entries[source_index.row()]
        if entry.value_type == "value_bool":
            return None

        editor = QLineEdit(parent)
        editor.setAlignment(Qt.AlignRight)

        regex_by_type = {
            "value_uint32": r"^\d+$",
            "value_int32": r"^-?\d+$",
            "value_float32": r"^-?\d*\.?\d+$",
        }
        regex = regex_by_type.get(entry.value_type)
        if regex is not None:
            editor.setValidator(QRegularExpressionValidator(QRegularExpression(regex), editor))

        return editor

    def paint(self, painter, option, index):
        if index.column() not in [ConfigTableModel.COL_REVERT, ConfigTableModel.COL_RESTORE_DEFAULT]:
            super().paint(painter, option, index)
            return

        painter.save()
        if option.state & QStyle.State_Selected:
            painter.fillRect(option.rect, option.palette.highlight())

        icon_rect = self._icon_rect(option.rect)
        if index.column() == ConfigTableModel.COL_REVERT:
            self.revert_icon.paint(painter, icon_rect, Qt.AlignCenter)
        else:
            self.default_icon.paint(painter, icon_rect, Qt.AlignCenter)
        painter.restore()

    def editorEvent(self, event, model, option, index):
        if index.column() not in [ConfigTableModel.COL_REVERT, ConfigTableModel.COL_RESTORE_DEFAULT]:
            return super().editorEvent(event, model, option, index)

        if event.type() != QEvent.MouseButtonRelease or event.button() != Qt.LeftButton:
            return False

        source_index = model.mapToSource(index) if hasattr(model, "mapToSource") else index
        manager = source_index.model().manager

        if not self._icon_rect(option.rect).contains(event.pos()):
            return False

        if index.column() == ConfigTableModel.COL_REVERT:
            manager.revert_entry(source_index.row())
            return True

        manager.restore_entry_default(source_index.row())
        return False

    def _icon_rect(self, cell_rect):
        icon_cell = QSize(26, 26)
        left = cell_rect.center().x() - icon_cell.width() // 2
        top = cell_rect.center().y() - icon_cell.height() // 2
        return QRect(left, top, icon_cell.width(), icon_cell.height())

    def setEditorData(self, editor, index):
        if editor is not None:
            editor.setText(index.data(Qt.EditRole))

    def setModelData(self, editor, model, index):
        if editor is not None:
            model.setData(index, editor.text(), Qt.EditRole)
