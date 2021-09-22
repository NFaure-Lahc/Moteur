# -*- coding: utf-8 -*-
"""
@author: Faure Nicolas.
Classe principale pour la gestion des moteurs.
Fais le lien entre la boite de dialogue et les fonctions.

Dernières modifications : 12/10/2020
"""

#%% Liste des importations
from PyQt5 import QtWidgets

# importe le widget moteur
from Form import Wid_DisplayMoteur as wid_moteur


#%% Classe principale de gestion des moteurs.
class Moteur(QtWidgets.QWidget): #, wid_moteur.Ui_moteur_template):
    """
    Initialisation du widget moteur.
    Effectue le lien entre les boutons du widget et les fonctions.
    """
    def __init__(self):
        super().__init__()

        self.IHM = wid_moteur.Ui_moteur_template()
        self.IHM.setupUi(self)


         # Fermeture de la BDLG
        self.IHM.But_Fermer.clicked.connect(self.close_window)

        # Homing
        self.IHM.But_Home.clicked.connect(self.home)

        # Déplacement relatif rapide
        self.IHM.But_FastPosStep.clicked.connect(self.fast_pos_step)
        self.IHM.But_FastNegStep.clicked.connect(self.fast_neg_step)

        # Déplacement relatif lent
        self.IHM.But_PosStep.clicked.connect(self.pos_step)
        self.IHM.But_NegStep.clicked.connect(self.neg_step)

        # Déplacement absolu
        self.IHM.But_Target.clicked.connect(self.target)

        # Change la vitesse de déplacement
        self.IHM.But_Velocity.clicked.connect(self.velocity)

    #%% show_window
    def show_window(self):
        """
        Affiche la fenêtre
        """
        self.show()

    #%% close_window
    def close_window(self):
        """
        Ferme la fenêtre.
        """
        self.close()

    #%% home
    def home(self):
        """
        Lance le Homing sur le moteur concerné.
        """

    #%% fast_pos_step
    def fast_pos_step(self):
        """
        Effectue un pas positif important
        """

    #%% fast_neg_step
    def fast_neg_step(self):
        """
        Effectue un pas négatif important
        """

    #%% pos_step
    def pos_step(self):
        """
        Effectue un pas positif
        """

    #%% neg_step
    def neg_step(self):
        """
        Effectue un pas négatif
        """

    #%% target
    def target(self):
        """
        Déplacement absolu vers la position TARGET
        """

    #%% velocity
    def velocity(self):
        """
        Régle la vitesse avec celle mise dans la BDLG
        """

    #%% update actual position display
    def update_actual_position_display(self, pos: float = 0):
        """
        Affiche la position actuelle
        """
        self.IHM.lcdNum_Position.display(pos)
        self.update()
        QtWidgets.QApplication.processEvents()

    #%% change_lang
    def change_lang(self):
        """
        Change les labels selon la langue sélectionnée et selon le moteur sélectionné
        """

    #%% move_to
    def move_to(self, target: int, lock_motor: bool = True):
        """
        Parameters
        ----------
        target : int
            Position à atteindre
        lock_motor : bool, optional
            Vérouille la fenêtre jusqu'à la fin du mouvement. The default is True.        """

    #%% is_moving ?
    def is_moving(self):
        """
        Vérifie si le moteur est en mouvement. Attends un msg de fin de mouvement
        """

    #%% init
    def init(self):
        """
        initialisation du moteur connecté
        """
