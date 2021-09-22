from __future__ import print_function
__signature__ = 0x59b6d589fbcaf327eedc4f70f9f539cc

# -*- coding: utf-8 -*-
"""
@author: Faure Nicolas
Classe pour la gestion des moteurs physics instrumente

Dernières modifications : 12/11/2020
"""

#%% Liste des importations
from time import sleep

from threading import Timer, Lock

import moteur
from physics_instrumente.pipython import GCSDevice, pitools



#%% Classe pour routine timeout
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
class Moteur_PI(moteur.Moteur):
    """
    Classe moteur thorlabs pour KCUBE et TCUBE
    """
    def __init__(self, serial: str, control_type: str, moteur_type: str, lang: str = "FR"):
        """
        Initialisation de la classe moteur_pi (fonctionne pour tous types de controlleur)
        """
        super().__init__()

        self.serial = serial
        self.control_type = control_type
        self.moteur_type = moteur_type
        self.lang = lang
        self.change_lang()

        self.control_type = control_type #'C-863'  # 'C-884' will also work
        self.stages = [moteur_type]
        self.REFMODES = ['FNL']


        """
        with GCSDevice(self.control_type) as self.pidevice:

            self.pidevice.ConnectUSB(serialnum=self.serial)
            print('connected: {}'.format(self.pidevice.qIDN().strip()))
            if self.pidevice.HasqVER():
                print('version info:\n{}'.format(self.pidevice.qVER().strip()))



            print('initialize connected stages...')
            pitools.startup(self.pidevice, stages=self.stages, refmodes=self.REFMODES)

            #self.update_position()
            #for axis in pidevice.axes:
            self.axis = self.pidevice.axes[0]
            #pidevice.GOH(axis)

            pos = self.pidevice.qPOS(self.axis)

            self.update_actual_position_display(pos[self.axis])
        """

        self.gcs = GCSDevice(devname=self.control_type)
        self.gcs.ConnectUSB(serialnum=self.serial)
        pitools.startup(self.gcs, stages=self.stages, refmodes=self.REFMODES)

        print (self.gcs.qIDN())

        print('version info:\n{}'.format(self.gcs.qVER().strip()))

        self.axis = self.gcs.axes[0]

        vel = self.gcs.qVEL(self.axis)
        print (vel[self.axis])

        self.IHM.lineEdit_Velocity.setText(str(round(vel[self.axis], 4)))

        self.IHM.lineEdit_InformationMoteur.setText(
                                "PI  " + self.control_type
                                +",    Moteur : " + self.moteur_type
                                +",    SN : " + self.serial)

        # Démarre une routine qui lit la position régulièrement (100ms).
        self.periodic = Periodic(0.1, self.update_position)


    #%% update_position
    def update_position(self):
        """
        Met à jour la position
        """
        pos = self.gcs.qPOS(self.axis)
        self.update_actual_position_display(pos[self.axis])


    #%% velocity
    def velocity(self):
        """
        Régle la vitesse avec celle mise dans la BDLG
        """
        #velmax = self.gcs.qVLS()
        #print (velmax)

        """if float(self.IHM.lineEdit_Velocity.text()) > velmax:
            self.gcs.VEL(self.axis, velmax)
            self.IHM.lineEdit_Velocity.setText(velmax)
        else :
            self.gcs.VEL(self.axis, float(self.IHM.lineEdit_Velocity.text()))
        """
        self.gcs.VEL(self.axis, float(self.IHM.lineEdit_Velocity.text()))


    #%% fast_pos_step
    def fast_pos_step(self):
        """
        Effectue un pas positif important
        """
        pos_act = self.gcs.qPOS(self.axis)

        pos_cib = pos_act[self.axis] + float(self.IHM.lineEdit_FastStepSize.text())
        self.move_to(pos_cib)


    #%% fast_neg_step
    def fast_neg_step(self):
        """
        Effectue un pas negatif important
        """
        pos_act = self.gcs.qPOS(self.axis)

        pos_cib = pos_act[self.axis] - float(self.IHM.lineEdit_FastStepSize.text())
        self.move_to(pos_cib)


    #%% pos_step
    def pos_step(self):
        """
        Effectue un pas positif
        """
        pos_act = self.gcs.qPOS(self.axis)

        pos_cib = pos_act[self.axis] + float(self.IHM.lineEdit_StepSize.text())
        self.move_to(pos_cib)


    #%% neg_step
    def neg_step(self):
        """
        Effectue un pas negatif
        """
        pos_act = self.gcs.qPOS(self.axis)

        pos_cib = pos_act[self.axis] - float(self.IHM.lineEdit_StepSize.text())
        self.move_to(pos_cib)


    # %% target
    def target(self):
        """
        Déplacement absolu vers la position TARGET
        """
        # Converti la position réelle en unité moteur
        self.move_to(float(self.IHM.lineEdit_Target.text()))


    #%% move_to
    def move_to(self, target: int, lock_motor: bool = True):
        self.gcs.MOV(self.axis, target)
        """
        Parameters
        ----------
        target : int
            Position à atteindre
        lock_motor : bool, optional
            Vérouille la fenêtre jusqu'à la fin du mouvement. The default is True.
        """


    #%% is_moving ?
    def is_moving(self):
        return self.gcs.IsMoving(self.axis)
        """
        Vérifie si le moteur est en mouvement. Attends un msg de fin de mouvement
        Returns
        -------
        None.
        """


    #%% home
    def home(self):
        """
        Lance le Homing sur le moteur concerné.
        """
        self.gcs.GOH()
        # Affiche la position finale ( doit être 0)
        self.update_position()

    #%% close
    def close_window(self):
        """
        Ferme les connexions avec le moteur
        """
        self.gcs.CloseConnection()
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

            self.IHM.label_5.setText("Pas (mm)")
            self.IHM.label_6.setText("Pos cible (mm)")
            self.IHM.label_7.setText("Vitesse (mm/s)")
            self.setWindowTitle("Moteur en translation")
            # Valeur par défaut step size, target
            self.IHM.lineEdit_Target.setText("5")
            self.IHM.lineEdit_StepSize.setText("0.1")
            self.IHM.lineEdit_FastStepSize.setText("5")
            self.IHM.But_Fermer.setText("Fermer")


        if self.lang == "EN":
            self.IHM.label.setText("Negative step")
            self.IHM.label_4.setText("Positive step")
            self.IHM.label_8.setText("Motor / controller informations")
            self.IHM.label_9.setText("Actual position")

            self.IHM.label_5.setText("Step (mm)")
            self.IHM.label_6.setText("Target (mm)")
            self.IHM.label_7.setText("Velocity (mm/s)")
            self.setWindowTitle("Translation motor")
            # Valeur par défaut step size, target
            self.IHM.lineEdit_Target.setText("5")
            self.IHM.lineEdit_StepSize.setText("0.1")
            self.IHM.lineEdit_FastStepSize.setText("5")
            self.IHM.But_Fermer.setText("Close")
