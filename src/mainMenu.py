# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainMenu.ui'
#
# Created: Tue Mar  6 15:28:36 2018
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(2700, 1280)
        self.verticalLayout = QtGui.QVBoxLayout(Form)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.btnNewGame = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        font.setBold(True)
        font.setWeight(75)
        self.btnNewGame.setFont(font)
        self.btnNewGame.setObjectName(_fromUtf8("btnNewGame"))
        self.verticalLayout.addWidget(self.btnNewGame)
        self.btnSettings = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        font.setBold(True)
        font.setWeight(75)
        self.btnSettings.setFont(font)
        self.btnSettings.setObjectName(_fromUtf8("btnSettings"))
        self.verticalLayout.addWidget(self.btnSettings)
        self.btnExit = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        font.setBold(True)
        font.setWeight(75)
        self.btnExit.setFont(font)
        self.btnExit.setObjectName(_fromUtf8("btnExit"))
        self.verticalLayout.addWidget(self.btnExit)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.btnNewGame.setText(_translate("Form", "New Game", None))
        self.btnSettings.setText(_translate("Form", "Settings", None))
        self.btnExit.setText(_translate("Form", "Exit", None))

