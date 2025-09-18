# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ui_main.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(1086, 662)
        self.textEdit_log = QTextEdit(Form)
        self.textEdit_log.setObjectName(u"textEdit_log")
        self.textEdit_log.setEnabled(True)
        self.textEdit_log.setGeometry(QRect(740, 220, 321, 421))
        self.textEdit_log.setMouseTracking(False)
        self.textEdit_log.setFocusPolicy(Qt.NoFocus)
#if QT_CONFIG(accessibility)
        self.textEdit_log.setAccessibleName(u"")
#endif // QT_CONFIG(accessibility)
#if QT_CONFIG(accessibility)
        self.textEdit_log.setAccessibleDescription(u"")
#endif // QT_CONFIG(accessibility)
        self.layoutWidget = QWidget(Form)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(11, 11, 867, 60))
        self.gridLayout = QGridLayout(self.layoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.label_dx = QLabel(self.layoutWidget)
        self.label_dx.setObjectName(u"label_dx")
        self.label_dx.setTextFormat(Qt.AutoText)
        self.label_dx.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_dx, 0, 0, 1, 1)

        self.label_dy = QLabel(self.layoutWidget)
        self.label_dy.setObjectName(u"label_dy")
        self.label_dy.setTextFormat(Qt.AutoText)
        self.label_dy.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_dy, 0, 1, 1, 1)

        self.label_dz = QLabel(self.layoutWidget)
        self.label_dz.setObjectName(u"label_dz")
        self.label_dz.setTextFormat(Qt.AutoText)
        self.label_dz.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_dz, 0, 2, 1, 1)

        self.label_drx = QLabel(self.layoutWidget)
        self.label_drx.setObjectName(u"label_drx")
        self.label_drx.setTextFormat(Qt.AutoText)
        self.label_drx.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_drx, 0, 3, 1, 1)

        self.label_dry = QLabel(self.layoutWidget)
        self.label_dry.setObjectName(u"label_dry")
        self.label_dry.setTextFormat(Qt.AutoText)
        self.label_dry.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_dry, 0, 4, 1, 1)

        self.label_drz = QLabel(self.layoutWidget)
        self.label_drz.setObjectName(u"label_drz")
        self.label_drz.setTextFormat(Qt.AutoText)
        self.label_drz.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_drz, 0, 5, 1, 1)

        self.lineEdit_dx = QLineEdit(self.layoutWidget)
        self.lineEdit_dx.setObjectName(u"lineEdit_dx")
        self.lineEdit_dx.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lineEdit_dx, 1, 0, 1, 1)

        self.lineEdit_dy = QLineEdit(self.layoutWidget)
        self.lineEdit_dy.setObjectName(u"lineEdit_dy")
        self.lineEdit_dy.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lineEdit_dy, 1, 1, 1, 1)

        self.lineEdit_dz = QLineEdit(self.layoutWidget)
        self.lineEdit_dz.setObjectName(u"lineEdit_dz")
        self.lineEdit_dz.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lineEdit_dz, 1, 2, 1, 1)

        self.lineEdit_drx = QLineEdit(self.layoutWidget)
        self.lineEdit_drx.setObjectName(u"lineEdit_drx")
        self.lineEdit_drx.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lineEdit_drx, 1, 3, 1, 1)

        self.lineEdit_dry = QLineEdit(self.layoutWidget)
        self.lineEdit_dry.setObjectName(u"lineEdit_dry")
        self.lineEdit_dry.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lineEdit_dry, 1, 4, 1, 1)

        self.lineEdit_drz = QLineEdit(self.layoutWidget)
        self.lineEdit_drz.setObjectName(u"lineEdit_drz")
        self.lineEdit_drz.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.lineEdit_drz, 1, 5, 1, 1)

        self.layoutWidget1 = QWidget(Form)
        self.layoutWidget1.setObjectName(u"layoutWidget1")
        self.layoutWidget1.setGeometry(QRect(10, 80, 730, 60))
        self.gridLayout_2 = QGridLayout(self.layoutWidget1)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.label_velx = QLabel(self.layoutWidget1)
        self.label_velx.setObjectName(u"label_velx")
        self.label_velx.setTextFormat(Qt.AutoText)
        self.label_velx.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.label_velx, 0, 0, 1, 1)

        self.label_velj = QLabel(self.layoutWidget1)
        self.label_velj.setObjectName(u"label_velj")
        self.label_velj.setTextFormat(Qt.AutoText)
        self.label_velj.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.label_velj, 0, 1, 1, 1)

        self.label_accx = QLabel(self.layoutWidget1)
        self.label_accx.setObjectName(u"label_accx")
        self.label_accx.setTextFormat(Qt.AutoText)
        self.label_accx.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.label_accx, 0, 2, 1, 1)

        self.label_accj = QLabel(self.layoutWidget1)
        self.label_accj.setObjectName(u"label_accj")
        self.label_accj.setTextFormat(Qt.AutoText)
        self.label_accj.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.label_accj, 0, 3, 1, 1)

        self.label_t = QLabel(self.layoutWidget1)
        self.label_t.setObjectName(u"label_t")
        self.label_t.setTextFormat(Qt.AutoText)
        self.label_t.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.label_t, 0, 4, 1, 1)

        self.lineEdit_velx = QLineEdit(self.layoutWidget1)
        self.lineEdit_velx.setObjectName(u"lineEdit_velx")
        self.lineEdit_velx.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.lineEdit_velx, 1, 0, 1, 1)

        self.lineEdit_velj = QLineEdit(self.layoutWidget1)
        self.lineEdit_velj.setObjectName(u"lineEdit_velj")
        self.lineEdit_velj.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.lineEdit_velj, 1, 1, 1, 1)

        self.lineEdit_accx = QLineEdit(self.layoutWidget1)
        self.lineEdit_accx.setObjectName(u"lineEdit_accx")
        self.lineEdit_accx.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.lineEdit_accx, 1, 2, 1, 1)

        self.lineEdit_accj = QLineEdit(self.layoutWidget1)
        self.lineEdit_accj.setObjectName(u"lineEdit_accj")
        self.lineEdit_accj.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.lineEdit_accj, 1, 3, 1, 1)

        self.lineEdit_t = QLineEdit(self.layoutWidget1)
        self.lineEdit_t.setObjectName(u"lineEdit_t")
        self.lineEdit_t.setAlignment(Qt.AlignCenter)

        self.gridLayout_2.addWidget(self.lineEdit_t, 1, 4, 1, 1)

        self.label_image = QLabel(Form)
        self.label_image.setObjectName(u"label_image")
        self.label_image.setGeometry(QRect(10, 220, 711, 421))
        self.label_image.setFrameShape(QFrame.Box)
        self.label_image.setFrameShadow(QFrame.Plain)
        self.layoutWidget2 = QWidget(Form)
        self.layoutWidget2.setObjectName(u"layoutWidget2")
        self.layoutWidget2.setGeometry(QRect(900, 11, 130, 170))
        self.verticalLayout = QVBoxLayout(self.layoutWidget2)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.pushButton_connect = QPushButton(self.layoutWidget2)
        self.pushButton_connect.setObjectName(u"pushButton_connect")

        self.verticalLayout.addWidget(self.pushButton_connect)

        self.pushButton_reset_status = QPushButton(self.layoutWidget2)
        self.pushButton_reset_status.setObjectName(u"pushButton_reset_status")

        self.verticalLayout.addWidget(self.pushButton_reset_status)

        self.pushButton_get_image = QPushButton(self.layoutWidget2)
        self.pushButton_get_image.setObjectName(u"pushButton_get_image")

        self.verticalLayout.addWidget(self.pushButton_get_image)

        self.pushButton_get_target_pos = QPushButton(self.layoutWidget2)
        self.pushButton_get_target_pos.setObjectName(u"pushButton_get_target_pos")

        self.verticalLayout.addWidget(self.pushButton_get_target_pos)

        self.pushButton_start_work = QPushButton(self.layoutWidget2)
        self.pushButton_start_work.setObjectName(u"pushButton_start_work")

        self.verticalLayout.addWidget(self.pushButton_start_work)

        self.layoutWidget3 = QWidget(Form)
        self.layoutWidget3.setObjectName(u"layoutWidget3")
        self.layoutWidget3.setGeometry(QRect(11, 157, 731, 25))
        self.gridLayout_3 = QGridLayout(self.layoutWidget3)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.pushButton_move_home = QPushButton(self.layoutWidget3)
        self.pushButton_move_home.setObjectName(u"pushButton_move_home")

        self.gridLayout_3.addWidget(self.pushButton_move_home, 0, 0, 1, 1)

        self.pushButton_get_pos = QPushButton(self.layoutWidget3)
        self.pushButton_get_pos.setObjectName(u"pushButton_get_pos")

        self.gridLayout_3.addWidget(self.pushButton_get_pos, 0, 1, 1, 1)

        self.pushButton_move_relative = QPushButton(self.layoutWidget3)
        self.pushButton_move_relative.setObjectName(u"pushButton_move_relative")

        self.gridLayout_3.addWidget(self.pushButton_move_relative, 0, 2, 1, 1)

        self.pushButton_move_work = QPushButton(self.layoutWidget3)
        self.pushButton_move_work.setObjectName(u"pushButton_move_work")

        self.gridLayout_3.addWidget(self.pushButton_move_work, 0, 3, 1, 1)

        self.pushButton_open_gripper = QPushButton(self.layoutWidget3)
        self.pushButton_open_gripper.setObjectName(u"pushButton_open_gripper")

        self.gridLayout_3.addWidget(self.pushButton_open_gripper, 0, 4, 1, 1)

        self.pushButton_close_gripper = QPushButton(self.layoutWidget3)
        self.pushButton_close_gripper.setObjectName(u"pushButton_close_gripper")

        self.gridLayout_3.addWidget(self.pushButton_close_gripper, 0, 5, 1, 1)

        self.pushButton_start_test = QPushButton(Form)
        self.pushButton_start_test.setObjectName(u"pushButton_start_test")
        self.pushButton_start_test.setGeometry(QRect(760, 90, 116, 23))
        self.pushButton_stop_test = QPushButton(Form)
        self.pushButton_stop_test.setObjectName(u"pushButton_stop_test")
        self.pushButton_stop_test.setGeometry(QRect(760, 120, 116, 23))
        self.pushButton_test = QPushButton(Form)
        self.pushButton_test.setObjectName(u"pushButton_test")
        self.pushButton_test.setGeometry(QRect(910, 190, 116, 23))
        self.lineEdit_debug_date = QLineEdit(Form)
        self.lineEdit_debug_date.setObjectName(u"lineEdit_debug_date")
        self.lineEdit_debug_date.setGeometry(QRect(760, 190, 139, 19))
        self.lineEdit_debug_date.setAlignment(Qt.AlignCenter)
        self.label_debug_date = QLabel(Form)
        self.label_debug_date.setObjectName(u"label_debug_date")
        self.label_debug_date.setGeometry(QRect(760, 160, 139, 33))
        self.label_debug_date.setTextFormat(Qt.AutoText)
        self.label_debug_date.setAlignment(Qt.AlignCenter)

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label_dx.setText(QCoreApplication.translate("Form", u"dx (mm)", None))
        self.label_dy.setText(QCoreApplication.translate("Form", u"dy (mm)", None))
        self.label_dz.setText(QCoreApplication.translate("Form", u"dz (mm)", None))
        self.label_drx.setText(QCoreApplication.translate("Form", u"drx (deg)", None))
        self.label_dry.setText(QCoreApplication.translate("Form", u"dry (deg)", None))
        self.label_drz.setText(QCoreApplication.translate("Form", u"drz (deg)", None))
        self.lineEdit_dx.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_dy.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_dz.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_drx.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_dry.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_drz.setText(QCoreApplication.translate("Form", u"0", None))
        self.label_velx.setText(QCoreApplication.translate("Form", u"velx (mm/s)", None))
        self.label_velj.setText(QCoreApplication.translate("Form", u"velj (deg/s)", None))
        self.label_accx.setText(QCoreApplication.translate("Form", u"accx (mm/s^2)", None))
        self.label_accj.setText(QCoreApplication.translate("Form", u"accj (deg/s^2)", None))
        self.label_t.setText(QCoreApplication.translate("Form", u"t (s)", None))
        self.lineEdit_velx.setText(QCoreApplication.translate("Form", u"100", None))
        self.lineEdit_velj.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_accx.setText(QCoreApplication.translate("Form", u"100", None))
        self.lineEdit_accj.setText(QCoreApplication.translate("Form", u"0", None))
        self.lineEdit_t.setText(QCoreApplication.translate("Form", u"0", None))
        self.label_image.setText("")
        self.pushButton_connect.setText(QCoreApplication.translate("Form", u"connect", None))
        self.pushButton_reset_status.setText(QCoreApplication.translate("Form", u"status_reset", None))
        self.pushButton_get_image.setText(QCoreApplication.translate("Form", u"get image", None))
        self.pushButton_get_target_pos.setText(QCoreApplication.translate("Form", u"get target pos", None))
        self.pushButton_start_work.setText(QCoreApplication.translate("Form", u"start_work", None))
        self.pushButton_move_home.setText(QCoreApplication.translate("Form", u"move_home", None))
        self.pushButton_get_pos.setText(QCoreApplication.translate("Form", u"get pos", None))
        self.pushButton_move_relative.setText(QCoreApplication.translate("Form", u"move_relative", None))
        self.pushButton_move_work.setText(QCoreApplication.translate("Form", u"move_work", None))
        self.pushButton_open_gripper.setText(QCoreApplication.translate("Form", u"open_gripper", None))
        self.pushButton_close_gripper.setText(QCoreApplication.translate("Form", u"close_gripper", None))
        self.pushButton_start_test.setText(QCoreApplication.translate("Form", u"start_test", None))
        self.pushButton_stop_test.setText(QCoreApplication.translate("Form", u"stop_test", None))
        self.pushButton_test.setText(QCoreApplication.translate("Form", u"TEST", None))
        self.lineEdit_debug_date.setText(QCoreApplication.translate("Form", u"20250912_022225", None))
        self.label_debug_date.setText(QCoreApplication.translate("Form", u"date", None))
    # retranslateUi

