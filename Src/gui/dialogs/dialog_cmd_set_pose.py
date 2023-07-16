

import pybullet as p
import pybullet_planning as pp
import numpy as np

from Src.Controller import Controller
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDialog


class UiDialogSetProbePosition(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Set Probe Position")
        Dialog.resize(371, 97)
        self.btn_drawpoint = QtWidgets.QPushButton(Dialog)
        self.btn_drawpoint.setGeometry(QtCore.QRect(200, 20, 75, 23))
        self.btn_drawpoint.setObjectName("btn_drawpoint")
        self.btn_setpos = QtWidgets.QPushButton(Dialog)
        self.btn_setpos.setGeometry(QtCore.QRect(200, 60, 75, 23))
        self.btn_setpos.setObjectName("btn_setpos")
        self.btn_cancel = QtWidgets.QPushButton(Dialog)
        self.btn_cancel.setGeometry(QtCore.QRect(290, 60, 75, 23))
        self.btn_cancel.setObjectName("btn_cancel")
        self.txt_x = QtWidgets.QLineEdit(Dialog)
        self.txt_x.setGeometry(QtCore.QRect(20, 20, 41, 20))
        self.txt_x.setText("")
        self.txt_x.setObjectName("txt_x")
        self.txt_y = QtWidgets.QLineEdit(Dialog)
        self.txt_y.setGeometry(QtCore.QRect(80, 20, 41, 20))
        self.txt_y.setText("")
        self.txt_y.setObjectName("txt_y")
        self.txt_z = QtWidgets.QLineEdit(Dialog)
        self.txt_z.setGeometry(QtCore.QRect(140, 20, 41, 20))
        self.txt_z.setText("")
        self.txt_z.setObjectName("txt_z")
        self.lbl_info_x = QtWidgets.QLabel(Dialog)
        self.lbl_info_x.setGeometry(QtCore.QRect(32, 4, 16, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_x.setFont(font)
        self.lbl_info_x.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_info_x.setObjectName("lbl_info_x")
        self.lbl_info_y = QtWidgets.QLabel(Dialog)
        self.lbl_info_y.setGeometry(QtCore.QRect(92, 2, 16, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_y.setFont(font)
        self.lbl_info_y.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_info_y.setObjectName("lbl_info_y")
        self.lbl_info_z = QtWidgets.QLabel(Dialog)
        self.lbl_info_z.setGeometry(QtCore.QRect(153, 2, 16, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_z.setFont(font)
        self.lbl_info_z.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_info_z.setObjectName("lbl_info_z")
        self.txt_ry = QtWidgets.QLineEdit(Dialog)
        self.txt_ry.setEnabled(True)
        self.txt_ry.setGeometry(QtCore.QRect(80, 61, 41, 20))
        self.txt_ry.setText("")
        self.txt_ry.setReadOnly(False)
        self.txt_ry.setPlaceholderText("")
        self.txt_ry.setObjectName("txt_ry")
        self.txt_rz = QtWidgets.QLineEdit(Dialog)
        self.txt_rz.setGeometry(QtCore.QRect(140, 61, 41, 20))
        self.txt_rz.setText("")
        self.txt_rz.setObjectName("txt_rz")
        self.lbl_info_z_2 = QtWidgets.QLabel(Dialog)
        self.lbl_info_z_2.setGeometry(QtCore.QRect(153, 43, 16, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_z_2.setFont(font)
        self.lbl_info_z_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_info_z_2.setObjectName("lbl_info_z_2")
        self.txt_rx = QtWidgets.QLineEdit(Dialog)
        self.txt_rx.setGeometry(QtCore.QRect(20, 61, 41, 20))
        self.txt_rx.setText("")
        self.txt_rx.setObjectName("txt_rx")
        self.lbl_info_y_2 = QtWidgets.QLabel(Dialog)
        self.lbl_info_y_2.setGeometry(QtCore.QRect(92, 43, 16, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_y_2.setFont(font)
        self.lbl_info_y_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_info_y_2.setObjectName("lbl_info_y_2")
        self.lbl_info_x_2 = QtWidgets.QLabel(Dialog)
        self.lbl_info_x_2.setGeometry(QtCore.QRect(32, 45, 16, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.lbl_info_x_2.setFont(font)
        self.lbl_info_x_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_info_x_2.setObjectName("lbl_info_x_2")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        Dialog.setTabOrder(self.txt_x, self.txt_y)
        Dialog.setTabOrder(self.txt_y, self.txt_z)
        Dialog.setTabOrder(self.txt_z, self.txt_rx)
        Dialog.setTabOrder(self.txt_rx, self.txt_ry)
        Dialog.setTabOrder(self.txt_ry, self.txt_rz)
        Dialog.setTabOrder(self.txt_rz, self.btn_drawpoint)
        Dialog.setTabOrder(self.btn_drawpoint, self.btn_setpos)
        Dialog.setTabOrder(self.btn_setpos, self.btn_cancel)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.btn_drawpoint.setText(_translate("Dialog", "Show Point"))
        self.btn_setpos.setText(_translate("Dialog", "Send"))
        self.btn_cancel.setText(_translate("Dialog", "Done"))
        self.lbl_info_x.setText(_translate("Dialog", "X"))
        self.lbl_info_y.setText(_translate("Dialog", "Y"))
        self.lbl_info_z.setText(_translate("Dialog", "Z"))
        self.lbl_info_z_2.setText(_translate("Dialog", "Rz"))
        self.lbl_info_y_2.setText(_translate("Dialog", "Ry"))
        self.lbl_info_x_2.setText(_translate("Dialog", "Rx"))


class DialogSetProbePosition(QDialog):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = UiDialogSetProbePosition()
        self.ui.setupUi(self)

        self.parent_instance = parent
        self.controller_instance: Controller = parent.controller
        self.ui.btn_setpos.clicked.connect(self.send_probe_state)
        self.ui.btn_drawpoint.clicked.connect(self.draw_point)
        self.ui.btn_cancel.clicked.connect(self.done)


    def draw_point(self):
        pos = [float(self.ui.txt_x.text()), float(self.ui.txt_y.text()), float(self.ui.txt_z.text())]
        p.addUserDebugPoints([pos], [[255, 0, 0]], 5,
                             replaceItemUniqueId=self.controller_instance.simulation_instance.debug_point)

    # [[x,y,z],[rx,ry,rz]]
    def send_probe_state(self):
        pos = [float(self.ui.txt_x.text()), float(self.ui.txt_y.text()), float(self.ui.txt_z.text())]
        #rot = [float(self.ui.txt_rx.text()) * (np.pi/180), float(self.ui.txt_ry.text()) * (np.pi/180), float(self.ui.txt_rz.text()) * (np.pi/180)]

