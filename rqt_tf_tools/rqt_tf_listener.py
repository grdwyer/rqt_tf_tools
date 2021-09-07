import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import rclpy.time
from time import sleep

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, Qt, Signal
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget, QLabel
from python_qt_binding.QtWidgets import QComboBox
from tf2_ros import TransformListener, Buffer, LookupException
import yaml
from PyKDL import Rotation
from math import degrees


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

        self.frames = []
        # TODO: param refresh rates
        self._widget.button_refresh_tf.clicked.connect(self.cb_refresh_list)
        self.time_tf_lookup = self._node.create_timer(0.1, self.cb_tf_lookup)

        self.cb_refresh_list()

    def cb_refresh_list(self):
        # This feels messy fully reloading each each update
        self._node.get_logger().debug("refresh tf list")
        frames = yaml.load(self.tf_buffer.all_frames_as_yaml(), yaml.FullLoader)
        p_index = self._widget.combo_parent.currentIndex()
        c_index = self._widget.combo_child.currentIndex()

        self._widget.combo_parent.clear()
        self._widget.combo_child.clear()
        self.frames.clear()

        for frame in frames:
            self.frames.append(frame)
            self._widget.combo_parent.addItem(frame)
            self._widget.combo_child.addItem(frame)

        self._widget.combo_parent.setCurrentIndex(p_index)
        self._widget.combo_child.setCurrentIndex(c_index)

    def cb_tf_lookup(self):
        # check items are selected or exist as frames
        if len(self.frames) < 1:
            self._node.get_logger().warn("no frames available")
            return

        parent = self.frames[self._widget.combo_parent.currentIndex()]
        child = self.frames[self._widget.combo_child.currentIndex()]
        self._node.get_logger().debug("Looking up between {} and {}".format(parent, child))
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                child,
                parent,
                now)
        except LookupException as ex:
            self._node.get_logger().error(
                f'Could not transform {child} to {parent}: {ex}')
            return

        self._widget.lcd_tx.display(trans.transform.translation.x)
        self._widget.lcd_ty.display(trans.transform.translation.y)
        self._widget.lcd_tz.display(trans.transform.translation.z)
        rotation = Rotation.Quaternion(trans.transform.rotation.x,
                                       trans.transform.rotation.y,
                                       trans.transform.rotation.z,
                                       trans.transform.rotation.w)
        rpy = rotation.GetRPY()
        self._widget.lcd_rx.display(degrees(rpy[0]))
        self._widget.lcd_ry.display(degrees(rpy[1]))
        self._widget.lcd_rz.display(degrees(rpy[2]))
