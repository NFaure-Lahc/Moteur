# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 16:25:15 2020

@author: Faure Nicolas
"""
#%% To do list
#
#
#   Event pour lecture de position qd rien ne se passe.



#%% Importation
import sys
from time import sleep

#"Sample code to get information about connected controllers."
from ctypes import (
    c_int,
    c_char_p,
    byref,
    c_uint16,
    c_uint32,
)

from PyQt5 import QtWidgets



# Importe le controle d'un moteur Thorlabs
from thorlabs_kinesis import KCube_DC_Servo as KCube
from thorlabs_kinesis import TCube_DC_Servo as TCube

from thorlabs_kinesis import device as dev



# importe la boite de dialogue principale
from Form import Wid_ControleMoteur_v2 as MainForm




#%% moteur_thorlabs
#
# classe principale du programme.
#
class moteur_thorlabs(QtWidgets.QWidget, MainForm.Ui_Moteur_Thorlabs):
    """
    Initialisation de la boite de dialogue moteur.
    Effectue le lien entre les boutons de la BDLG et les fonctions.
    """
    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        self.setupUi(self)

        # Fermeture de la BDLG
        self.But_Fermer.clicked.connect(    self.fermer     )

        # Homing
        self.But_Home.clicked.connect(      self.home       )

        # Déplacement vers les limites
        self.But_FastPosStep.clicked.connect(  self.fast_pos_step   )
        self.But_FastNegStep.clicked.connect(  self.fast_neg_step   )

        # Déplacement relatif
        self.But_PosStep.clicked.connect(   self.pos_step    )
        self.But_NegStep.clicked.connect(   self.neg_step    )

        # Déplacement absolu
        self.But_Target.clicked.connect(    self.target     )
        # Change la vitesse de déplacement
        self.But_Velocity.clicked.connect(  self.velocity   )


    # %% Fermer
    def fermer(self):
        """
        Ferme la connexion avec le moteur
        Ferme la fenêtre.
        """
        if self.obj.CC_CheckConnection(self.SN_Device):
            self.obj.CC_ClearMessageQueue(self.SN_Device)
            self.obj.CC_StopPolling(self.SN_Device)
            self.obj.CC_Close(self.SN_Device)
            print("Fermeture de la connexion")
        self.close()

    # %% change_label_fr
    def change_label_fr(self):
        """
        Change les labels en FR de la fenêtre selon le moteur sélectionné
        """
        self.label.setText("Pas négatif")
        self.label_4.setText("Pas positif")
        self.label_8.setText("Information sur le moteur / controleur")
        self.label_9.setText("Position actuelle")

        if self.Motortype == "PRM1-Z8":
            self.label_5.setText("Pas (°)")
            self.label_6.setText("Pos cible (°)")
            self.label_7.setText("Vitesse (°/s)")
            self.setWindowTitle("Moteur en rotation")

            # Valeur par défaut step size, target
            self.lineEdit_Target.setText("10")
            self.lineEdit_StepSize.setText("0.1")
            self.lineEdit_FastStepSize.setText("5")


        if ((self.Motortype == "MTS50-Z8") or (self.Motortype == "MTS25-Z8")):
            self.label_5.setText("Pas (mm)")
            self.label_6.setText("Pos cible (mm)")
            self.label_7.setText("Vitesse (mm/s)")
            self.setWindowTitle("Moteur en translation")
            # Valeur par défaut step size, target
            self.lineEdit_Target.setText("5")
            self.lineEdit_StepSize.setText("0.1")
            self.lineEdit_FastStepSize.setText("5")




    # %% target
    def target(self):
        """
        Déplacement absolu vers la position TARGET
        """
        print("Target ...")

        # Converti la position réelle en unité moteur
        pos_cib = dev.real_to_device_units(self.Motortype,
                                               float(self.lineEdit_Target.text()), "position")

        # Envoi l'ordre de se déplacer
        self.move_to(pos_cib)

        print("... Fin target")

    #%% move_to
    def move_to(self, target:int, lock_motor:bool = True):
        """
        Parameters
        ----------
        target : int
            Position à atteindre
        lock_motor : bool, optional
            Vérouille la fenêtre jusqu'à la fin du mouvement. The default is True.

        Returns
        -------
        None.
        """
        # Vérification de la connexion ( = TRUE)
        if self.obj.CC_CheckConnection(self.SN_Device):

            self.obj.CC_ClearMessageQueue(self.SN_Device)
            sleep(0.1)

            # Envoi de la position cible au moteur
            self.obj.CC_SetMoveAbsolutePosition(self.SN_Device, target)
            sleep(0.1)

            # Envoi de l'ordre de déplacement
            self.obj.CC_MoveAbsolute(self.SN_Device)
            sleep(0.1)

            # Bloque le programme jusqu'à fin du mouvement
            if lock_motor:
                self.is_moving()


    #%% is_moving ?
    def is_moving(self):
        """
        Vérifie si le moteur est en mouvement. Attends un msg de fin de mouvement

        Returns
        -------
        None.
        """
        moved = False
        msgType, msgID, msgData = c_uint16(), c_uint16(), c_uint32()

        while not moved:
            if self.obj.CC_MessageQueueSize(self.SN_Device) != 0: # read the Serial Meassage Queue
                self.obj.CC_GetNextMessage(self.SN_Device, byref(msgType), byref(msgID), byref(msgData))
                #print(msgType, msgID, msgData, device_msg[msgType.value][0], device_msg[msgType.value][1][msgID.value])
                #if msgType.value == 2 and msgID.value == 0: # device is done homing
                pos = int(self.obj.CC_GetPosition(self.SN_Device))
                self.display_actual_position(pos)

                if msgType.value == 2 and msgID.value == 1: # device has moved
                    moved = True
                if msgType.value == 2 and msgID.value == 2: # device has been stopped
                    moved = True

    #%% velocity
    def velocity(self):
        """
        Régle la vitesse avec celle mise dans la BDLG

        Returns
        -------
        None.
        """

        print("Début velocity ...")

        acceleration = c_int(0)
        max_velocity = c_int(0)


        # Lit la valeur d'origine
        self.obj.CC_RequestVelParams(self.SN_Device)
        self.obj.CC_GetVelParams(self.SN_Device,
                                 byref(acceleration),
                                 byref(max_velocity))

        # Nouvelle valeur de max_velocity à appliquer
        max_velocity.value = dev.real_to_device_units(self.Motortype,
                                                      float(self.lineEdit_Velocity.text()),
                                                      "velocity")

        # Met à jour la valeur
        self.obj.CC_SetVelParams(self.SN_Device,
                                 (acceleration),
                                 (max_velocity))


        print("... Fin velocity")


    #%% init
    def init(self):
        """
        initialisation du moteur connecté
        """
        milliseconds = c_int(100)

        # création de la liste des moteurs connectés.
        self.obj.TLI_BuildDeviceList()

        # Ouverture de la connection
        if self.obj.CC_Open(self.SN_Device) == 0:
            # identification du controleur en cours
            self.obj.CC_Identify(self.SN_Device)


            self.obj.CC_StartPolling(self.SN_Device, milliseconds)
            self.obj.CC_ClearMessageQueue(self.SN_Device)
            sleep(1.0)

            self.display_actual_position(int(self.obj.CC_GetPosition(self.SN_Device)))

            # récupération des infos controleur + moteur
            hw_info = self.obj.TLI_HardwareInformation()  # container for hw info
            self.obj.CC_GetHardwareInfoBlock(self.SN_Device, byref(hw_info))

            type_controleur = ""
            if hw_info.modelNumber == b'KDC101':
                type_controleur = "KDC101"
            if hw_info.modelNumber == b'TDC001':
                type_controleur = "TDC001"


            self.lineEdit_InformationMoteur.setText(
                                type_controleur
                                +",    SN : " + str(hw_info.serialNumber)
                                +",    Moteur : " + self.Motortype)

            # Lecture de la valeur actuelle de Vitesse
            acceleration = c_int(0)
            max_velocity = c_int(0)

            self.obj.CC_RequestVelParams(self.SN_Device)
            self.obj.CC_GetVelParams(self.SN_Device,
                                     byref(acceleration),
                                     byref(max_velocity))

            # Conversion valeur lue en unité rééle
            max_vel = dev.device_to_real_units(self.Motortype,
                                               float(max_velocity.value), "velocity")

            # Affichage avec arrondi à la quatrième décimale
            self.lineEdit_Velocity.setText(str(round(max_vel, 4)))

            # Met à jour la face avant
            self.change_label_fr()

            return True
        return False




    #%% set_param
    def set_param(self, SN: int, motor_type: str):
        """
        Envoi les paramêtes sur le type de moteur et de contrôleur
        """
        self.SN_Device = c_char_p(bytes(SN, "utf-8"))
        self.Motortype = motor_type

        if dev.expand_device(SN).type == "KCube DC Servo":
            self.obj = KCube
        if dev.expand_device(SN).type == "TCube DC Servo":
            self.obj = TCube


    #%% home
    def home(self):
        """
        Lance le Homing sur le moteur concerné.
        """
        print("Homing ...")

        # Vérification de la connexion
        if self.obj.CC_CheckConnection(self.SN_Device):

            #
            self.obj.CC_ClearMessageQueue(self.SN_Device)
            sleep(1.0)

            # Lance l'ordre du Homing
            err = self.obj.CC_Home(self.SN_Device)
            sleep(1.0)

            # Si Homing en cours :
            if err == 0:
                while True:
                    # Lecture de la position actuelle
                    current_pos = int(self.obj.CC_GetPosition(self.SN_Device))

                    # Si la pos actuelle est 0, le homing est fini.
                    if current_pos == 0:
                        break
                    # Affiche la position actuelle
                    self.display_actual_position(current_pos)


                    sleep(0.2)
            else:
                print(f"Can't home. Err: {err}")

        else:
            print("Connexion non valide")


        # Affiche la position finale ( doit être 0)
        pos = int(self.obj.CC_GetPosition(self.SN_Device))
        self.display_actual_position(pos)


        print("... Fin Homing")
        print("")

    #%% fast_pos_step
    def fast_pos_step(self):
        """
        Effectue un pas positif important
        """
        print("Début FastPosStep ...")

        if self.obj.CC_CheckConnection(self.SN_Device):
            # position actuelle
            pos_act = int(self.obj.CC_GetPosition(self.SN_Device))


            pos_cib = pos_act + dev.real_to_device_units(self.Motortype,
                                                         float(self.lineEdit_FastStepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

        print("... Fin FastPosStep")
        print("")

    #%% fast_neg_step
    def fast_neg_step(self):
        """
        Effectue un pas négatif important
        """
        print("Début FastNegStep ...")

        if self.obj.CC_CheckConnection(self.SN_Device):
            # position actuelle
            pos_act = int(self.obj.CC_GetPosition(self.SN_Device))


            pos_cib = pos_act - dev.real_to_device_units(self.Motortype,
                                                         float(self.lineEdit_FastStepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

        print("... Fin FastNegStep")
        print("")

    #%% pos_step
    def pos_step(self):
        """
        Effectue un pas positif
        """
        print("Début PosStep ...")

        if self.obj.CC_CheckConnection(self.SN_Device):
            # position actuelle
            pos_act = int(self.obj.CC_GetPosition(self.SN_Device))


            pos_cib = pos_act + dev.real_to_device_units(self.Motortype,
                                                         float(self.lineEdit_StepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

        print("... Fin PosStep")
        print("")

    #%% neg_step
    def neg_step(self):
        """
        Effectue un pas négatif
        """
        print("Début NegStep ...")

        if self.obj.CC_CheckConnection(self.SN_Device):
            # position actuelle
            pos_act = int(self.obj.CC_GetPosition(self.SN_Device))


            pos_cib = pos_act - dev.real_to_device_units(self.Motortype,
                                                         float(self.lineEdit_StepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

        print("... Fin NegStep")
        print("")

    #%% display_actual_position
    def display_actual_position(self, pos):
        """
        Affiche la position actuelle
        """

        # Converti la valeur de device vers valeur réelle.
        pos_deg = dev.device_to_real_units(self.Motortype, pos, "position")

        # Affiche la valeur
        self.lcdNum_Position.display(pos_deg)

        # Permet l'affichage en live de la valeur.
        self.update()
        QtWidgets.QApplication.processEvents()


#%% main
def main():
    """
    Fonction principale si ce fichier est exécuté directement.
    """

    app = QtWidgets.QApplication.instance()   # checks if QApplication already exists
    if not app:                             # create QApplication if it doesnt exist
        app = QtWidgets.QApplication(sys.argv)


    main_mot_2 = moteur_thorlabs()
    main_mot_2.set_param("27002258", "PRM1-Z8")
    if main_mot_2.init():
        main_mot_2.show()

    """
    main_mot_ = moteur_thorlabs()
    main_mot_.set_param("83827930", "MTS50-Z8")
    if main_mot_.init():
        main_mot_.show()
    """




    QtWidgets.QApplication.setQuitOnLastWindowClosed(True)
    app.exec_()
    app.quit()

    #%%

#if __name__ == ' __main__ ':
main()


# Numéro de série du controlleur PRM1-Z8        MTS50-Z8
#SN_Thorlabs = "27002258"   # KDC101
#SN_Thorlabs = "83827930"   # TDC100
#SN_Thorlabs = "83849932"   # TDC100
