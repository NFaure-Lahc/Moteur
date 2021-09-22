# -*- coding: utf-8 -*-
"""
Created on Wed Oct  7 12:43:42 2020

@author: fani9660
"""

import sys
from PyQt5 import QtWidgets

import moteur_thorlabs_KCube
import moteur_thorlabs_TCube

import moteur_pi

# Create Qt application
app = QtWidgets.QApplication.instance()   # checks if QApplication already exists
if not app:                             # create QApplication if it doesnt exist
    app = QtWidgets.QApplication(sys.argv)



# Crée l'objet principal, avec un lien vers une fenêtre.
#mot = moteur_thorlabs_KCube.MoteurThorlabs_KCube("27002258", "KDC101", "MTS50-Z8", "FR")
#mot.show_window() # Affiche la boite de dialogue, mais pas obligatoire pour utiliser les fonctions.

mot2 = moteur_pi.Moteur_PI("0125500356", "C-863", "M-126.DG1", "FR")
mot2.show_window() # Affiche la boite de dialogue, mais pas obligatoire pour utiliser les fonctions.





QtWidgets.QApplication.setQuitOnLastWindowClosed(True)
app.exec_()
app.quit()

#83849932
