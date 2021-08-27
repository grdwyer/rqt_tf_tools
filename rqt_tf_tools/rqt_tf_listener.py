import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from time import sleep

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel
from python_qt_binding.QtSvg import QSvgRenderer
from tf2_ros import TransformListener, Buffer, LookupException


class RqtTFListener(Plugin):
    def __init__(self, context):
        super(RqtTFListener, self).__init__(context)
        self._widget = QWidget()
        self._node = context.node

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)

        ui_file = os.path.join(get_package_share_directory('rqt_tf_tools'), 'resource', 'tf_listener.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TF Listener')
        self.setObjectName('TF Listener')
        self._widget.setWindowTitle('TF Listener')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + ('(%d)' % context.serial_number()))
            # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.button_refresh_list.clicked.connect(self.cb_refresh_list)

    def cb_refresh_list(self):
        self.tf_buffer.all_frames_as_string()