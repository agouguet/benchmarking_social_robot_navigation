import os, sys, math, random, copy, yaml, cv2

from PyQt5.uic import loadUi
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import*

class AddHumanNodeDlg(QDialog):
    """Add Human Node dialog."""
    def __init__(self, model, clicked_node, parent=None):
        super().__init__(parent)
        # Load the dialog's GUI
        loadUi("ui/add_human_node_box.ui", self)

        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.model = model
        self.clicked_node = clicked_node

        self.init_combo_boxes()


    def init_combo_boxes(self):
        for n in self.model.G.nodes():
            item = str(n)
            self.startNodeComboBox.addItem(item)
            self.endNodeComboBox.addItem(item)

        self.startNodeComboBox.setCurrentText(str(self.clicked_node))

    def get_values_of_nodes(self):
        return (int(self.startNodeComboBox.currentText()), int(self.endNodeComboBox.currentText()))