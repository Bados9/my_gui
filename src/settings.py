# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'settings.ui'
#
# Created: Tue Mar 13 16:44:11 2018
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
        self.gridLayout = QtGui.QGridLayout(Form)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.pushButton = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.gridLayout.addWidget(self.pushButton, 0, 0, 1, 1)
        self.pushButton_2 = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.gridLayout.addWidget(self.pushButton_2, 1, 0, 1, 1)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.btnApply = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        self.btnApply.setFont(font)
        self.btnApply.setObjectName(_fromUtf8("btnApply"))
        self.horizontalLayout.addWidget(self.btnApply)
        self.btnBack = QtGui.QPushButton(Form)
        font = QtGui.QFont()
        font.setPointSize(72)
        self.btnBack.setFont(font)
        self.btnBack.setObjectName(_fromUtf8("btnBack"))
        self.horizontalLayout.addWidget(self.btnBack)
        self.gridLayout.addLayout(self.horizontalLayout, 3, 1, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.pushButton.setText(_translate("Form", "nastaveni cehosi", None))
        self.pushButton_2.setText(_translate("Form", "dalsi nastaveni", None))
        self.btnApply.setText(_translate("Form", "Apply", None))
        self.btnBack.setText(_translate("Form", "Back", None))

