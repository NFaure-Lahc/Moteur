# -*- coding: utf-8 -*-
"""
Created on Wed Sep 23 11:30:02 2020

@author: Faure Nicolas

"""


from PyQt5 import QtWidgets
import sys


import moteur_thorlabs as mt
#from Form import Wid_ControleMoteur_v3 as wid_mot


# importe la boite de dialogue principale
from Form import MainWid as MainWin


class MainWindo(QtWidgets.QWidget, MainWin.Ui_Form):
    #
    #   Initialisation de la boite de dialogue moteur.
    #   Effectue le lien entre les boutons de la BDLG et les fonctions.
    #
    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.setupUi(self)

        # Clic Moteur
        self.But_Moteur1.clicked.connect(  self.Mot1   )
        self.But_Moteur2.clicked.connect(  self.Mot2   )

    def Mot1(self):
        print (self.Edit_SN_Moteur1.text())
        MainMot = mt.moteur_thorlabs()
        MainMot.set_param("27002258", "PRM1-Z8")
        if ( MainMot.init() ) :
            MainMot.show()

    def Mot2(self):
        print (self.Edit_SN_Moteur2.text())



def main():
    # checks if QApplication already exists
    app=QtWidgets.QApplication.instance()

    # create QApplication if it doesnt exist
    if not app:
        app = QtWidgets.QApplication(sys.argv)


    MainWindow = MainWindo()
    MainWindow.show()








    #
    QtWidgets.QApplication.setQuitOnLastWindowClosed(True)
    app.exec_()
    app.quit()



#if __name__ == ' __main__ ':
main()