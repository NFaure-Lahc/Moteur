# -*- coding: utf-8 -*-
"""
@author: Faure Nicolas
Classe pour la gestion des moteurs thorlabs

Dernières modifications : 12/10/2020
"""

#%% Liste des importations
from time import sleep
from ctypes import (
    c_int,
)

from threading import Timer, Lock

import moteur
from thorlabs_kinesis import KCube_DC_Servo_Fonction as KCube
from thorlabs_kinesis import device as dev

class Periodic(object):
    """
    A periodic task running in threading.Timers
    """

    def __init__(self, interval, function, *args, **kwargs):
        self._lock = Lock()
        self._timer = None
        self.function = function
        self.interval = interval
        self.args = args
        self.kwargs = kwargs
        self._stopped = True
        if kwargs.pop('autostart', True):
            self.start()

    def start(self, from_run=False):
        self._lock.acquire()
        if from_run or self._stopped:
            self._stopped = False
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self._lock.release()

    def _run(self):
        self.start(from_run=True)
        self.function(*self.args, **self.kwargs)

    def stop(self):
        self._lock.acquire()
        self._stopped = True
        self._timer.cancel()
        self._lock.release()



#%% Classe Héritière de la classe Moteur, dans moteur.py
class MoteurThorlabs_KCube(moteur.Moteur):
    """
    Classe moteur thorlabs pour KCUBE et TCUBE
    """
    def __init__(self, serial: str, control_type: str, moteur_type: str, lang: str = "FR"):
        """
        Initialisation de la classe moteur_thorlabs pour KCUBE ou TCUBE.
        """
        super().__init__()

        self.serial = serial
        self.control_type = control_type
        self.moteur_type = moteur_type
        self.lang = lang
        self.change_lang()


        #initialisation du moteur connecté
        milliseconds = c_int(100)

        # création de la liste des moteurs connectés.
        self.KCube = KCube.KCubeDCServoFonction()
        self.KCube.__init___(self.serial)
        self.KCube.TLI_BuildDeviceList()

        # Ouverture de la connection
        if self.KCube.open() == 0:
            # identification du controleur en cours (clignotement)
            self.KCube.identify()

            self.KCube.start_polling(milliseconds)
            self.KCube.clear_message_queue()
            sleep(1.0)

            #pos = dev.device_to_real_units(self.moteur_type,
              #                             self.KCube.get_position(),
               #                            "position")
            self.update_position()

            self.IHM.lineEdit_InformationMoteur.setText(
                                "Thorlabs  " + self.control_type
                                +",    Moteur : " + self.moteur_type
                                +",    SN : " + self.serial)


            # Lecture de la valeur actuelle de Vitesse
            self.KCube.request_vel_params()
            [vel, acc] = self.KCube.get_vel_params()

            # Conversion valeur lue en unité réelle
            max_vel = dev.device_to_real_units(self.moteur_type,
                                               float(vel), "velocity")

            # Affichage avec arrondi à la quatrième décimale
            self.IHM.lineEdit_Velocity.setText(str(round(max_vel, 4)))

            # Démarre une routine qui lit la position régulièrement (100ms).
            self.periodic = Periodic(0.1, self.update_position) # it auto-starts, no need of rt.start()


    #%% update_position
    def update_position(self):
        pos = dev.device_to_real_units(self.moteur_type,
                                       int(self.KCube.get_position()),
                                       "position")
        if self.moteur_type == "PRM1-Z8":
            if pos < 0: pos += 360
            if pos > 360: pos -= 360
        self.update_actual_position_display(pos)

    #%% velocity
    def velocity(self):
        """
        Régle la vitesse avec celle mise dans la BDLG
        """
        # Lit la valeur d'origine
        self.KCube.request_vel_params()
        [vel, acc] = self.KCube.get_vel_params()

        # Nouvelle valeur de max_velocity à appliquer
        vel = dev.real_to_device_units(self.moteur_type,
                                       float(self.IHM.lineEdit_Velocity.text()),
                                       "velocity")

        # Met à jour la valeur
        self.KCube.set_vel_params(vel, acc)

        [vel, acc] = self.KCube.get_vel_params()

    #%% fast_pos_step
    def fast_pos_step(self):
        """
        Effectue un pas positif important
        """
        if self.KCube.check_connection():
            # position actuelle
            pos_act = int(self.KCube.get_position())

            pos_cib = pos_act + dev.real_to_device_units(self.moteur_type,
                                                         float(self.IHM.lineEdit_FastStepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

    #%% fast_neg_step
    def fast_neg_step(self):
        """
        Effectue un pas negatif important
        """
        if self.KCube.check_connection():
            # position actuelle
            pos_act = int(self.KCube.get_position())

            pos_cib = pos_act - dev.real_to_device_units(self.moteur_type,
                                                         float(self.IHM.lineEdit_FastStepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

    #%% pos_step
    def pos_step(self):
        """
        Effectue un pas positif
        """
        if self.KCube.check_connection():
            # position actuelle
            pos_act = int(self.KCube.get_position())

            pos_cib = pos_act + dev.real_to_device_units(self.moteur_type,
                                                         float(self.IHM.lineEdit_StepSize.text()),
                                                         "position")
            self.move_to(pos_cib)

    #%% neg_step
    def neg_step(self):
        """
        Effectue un pas negatif
        """
        if self.KCube.check_connection():
            # position actuelle
            pos_act = int(self.KCube.get_position())

            pos_cib = pos_act - dev.real_to_device_units(self.moteur_type,
                                                         float(self.IHM.lineEdit_StepSize.text()),
                                                         "position")
            self.move_to(pos_cib)


    # %% target
    def target(self):
        """
        Déplacement absolu vers la position TARGET
        """
        # Converti la position réelle en unité moteur
        pos_cib = dev.real_to_device_units(self.moteur_type,
                                           float(self.IHM.lineEdit_Target.text()),
                                           "position")

        # Envoi l'ordre de se déplacer
        self.move_to(pos_cib)


    #%% move_to
    def move_to(self, target: int, lock_motor: bool = True):
        """
        Parameters
        ----------
        target : int
            Position à atteindre
        lock_motor : bool, optional
            Vérouille la fenêtre jusqu'à la fin du mouvement. The default is True.
        """
        # Vérification de la connexion ( = TRUE)
        if self.KCube.check_connection():
            self.KCube.clear_message_queue()
            sleep(0.1)

            # Envoi de la position cible au moteur
            self.KCube.set_move_absolute_position(target)
            sleep(0.1)

            # Envoi de l'ordre de déplacement
            self.KCube.move_absolute()
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

        while not moved:
            if self.KCube.message_queue_size() != 0: # read the Serial Meassage Queue
                [msg_type, msg_id, msg_data] = self.KCube.get_next_message()
                #print(msgType, msgID, msgData, device_msg[msgType.value][0], device_msg[msgType.value][1][msgID.value])
                #if msgType.value == 2 and msgID.value == 0: # device is done homing
                self.update_position()

                if msg_type == 2 and msg_id == 1: # device has moved
                    moved = True
                if msg_type == 2 and msg_id == 2: # device has been stopped
                    moved = True


    #%% home
    def home(self):
        """
        Lance le Homing sur le moteur concerné.
        """
        # Vérification de la connexion
        if self.KCube.check_connection():
            #
            self.KCube.clear_message_queue()
            sleep(1.0)

            # Lance l'ordre du Homing
            err = self.KCube.home()
            sleep(1.0)

            # Si Homing en cours :
            if err == 0:
                while True:
                    # Lecture de la position actuelle
                    current_pos = int(self.KCube.get_position())

                    # Si la pos actuelle est 0, le homing est fini.
                    if current_pos == 0:
                        break
                    # Affiche la position actuelle
                    self.update_position()
                    sleep(0.2)
            else:
                print(f"Can't home. Err: {err}")
        else:
            print("Connexion non valide")


        # Affiche la position finale ( doit être 0)
        self.update_position()

    #%% close
    def close_window(self):
        """
        Ferme les connexions avec le moteur
        """
        self.KCube.close()
        self.periodic.stop()
        super().close_window()

    #%% change_lang
    def change_lang(self):
        """
        Change les labels selon la langue sélectionnée et selon le moteur sélectionné
        """
        if self.lang == "FR":
            self.IHM.label.setText("Pas négatif")
            self.IHM.label_4.setText("Pas positif")
            self.IHM.label_8.setText("Informations sur le moteur / controleur")
            self.IHM.label_9.setText("Position actuelle")

            if self.moteur_type == "PRM1-Z8":
                self.IHM.label_5.setText("Pas (°)")
                self.IHM.label_6.setText("Pos cible (°)")
                self.IHM.label_7.setText("Vitesse (°/s)")
                self.setWindowTitle("Moteur en rotation")

                # Valeur par défaut step size, target
                self.IHM.lineEdit_Target.setText("10")
                self.IHM.lineEdit_StepSize.setText("0.1")
                self.IHM.lineEdit_FastStepSize.setText("5")

            if self.moteur_type == "MTS50-Z8" or self.moteur_type == "MTS25-Z8":
                self.IHM.label_5.setText("Pas (mm)")
                self.IHM.label_6.setText("Pos cible (mm)")
                self.IHM.label_7.setText("Vitesse (mm/s)")
                self.setWindowTitle("Moteur en translation")
                # Valeur par défaut step size, target
                self.IHM.lineEdit_Target.setText("5")
                self.IHM.lineEdit_StepSize.setText("0.1")
                self.IHM.lineEdit_FastStepSize.setText("5")


        if self.lang == "EN":
            self.IHM.label.setText("Negative step")
            self.IHM.label_4.setText("Positive step")
            self.IHM.label_8.setText("Motor / controller informations")
            self.IHM.label_9.setText("Actual position")

            if self.moteur_type == "PRM1-Z8":
                self.IHM.label_5.setText("Step (°)")
                self.IHM.label_6.setText("Target (°)")
                self.IHM.label_7.setText("Velocity (°/s)")
                self.setWindowTitle("Rotation motor")

                # Valeur par défaut step size, target
                self.IHM.lineEdit_Target.setText("10")
                self.IHM.lineEdit_StepSize.setText("0.1")
                self.IHM.lineEdit_FastStepSize.setText("5")

            if self.moteur_type == "MTS50-Z8" or self.moteur_type == "MTS25-Z8":
                self.IHM.label_5.setText("Step (mm)")
                self.IHM.label_6.setText("Target (mm)")
                self.IHM.label_7.setText("Velocity (mm/s)")
                self.setWindowTitle("Translation motor")
                # Valeur par défaut step size, target
                self.IHM.lineEdit_Target.setText("5")
                self.IHM.lineEdit_StepSize.setText("0.1")
                self.IHM.lineEdit_FastStepSize.setText("5")
