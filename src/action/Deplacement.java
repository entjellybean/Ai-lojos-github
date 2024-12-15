package action;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

/**
 * La classe Deplacement permet de contrôler les déplacements d'un robot 
 * équipé de deux roues motrices en utilisant la bibliothèque LeJOS EV3.
 *  @author Yassmina Cherqaoui
 */

public class Deplacement {

    
    private MovePilot moteurPilotage; // Instance du pilot pour contrôler les mouvements du robot
    private double orientation;   //permet de connaître l'orientation du robot par rapport à sa position de base

    /**
     * Constructeur de la classe Deplacement.
     * Configure les moteurs, les roues, et initialise les paramètres du pilot.
     *
     * @param A Port pour le moteur gauche.
     * @param C Port pour le moteur droit.
     */
    public Deplacement(Port A,Port C) {
        
    	EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(A);
    	EV3LargeRegulatedMotor motorC = new EV3LargeRegulatedMotor(C);
    	Wheel roueDroite = WheeledChassis.modelWheel(motorA, 5.6).offset(-6.850); // Configuration de la roue gauche
        Wheel roueGauche = WheeledChassis.modelWheel(motorC, 5.6).offset(6.850);   // Configuration de la roue droite
        Chassis baseRoues = new WheeledChassis(new Wheel[] {roueGauche, roueDroite}, WheeledChassis.TYPE_DIFFERENTIAL);
        moteurPilotage = new MovePilot(baseRoues);
        moteurPilotage.setLinearSpeed(27); // Vitesse linéaire définie à 27 cm/s
        moteurPilotage.setAngularSpeed(30); // Vitesse angulaire en degrés/s
        orientation = 0;
    }

    /**
     * Met à jour l'orientation du robot.
     *
     * @param nouvelAngle Angle en degrés à définir comme orientation actuelle.
     */

    public void setOrientation(double nouvelAngle) {
        orientation = (nouvelAngle + 360) % 360; 
    }

    /**
     * Définit la vitesse de rotation du robot.
     *
     * @param speed Vitesse de rotation en degrés par seconde.
     */
    public void setAngularSpeed(int speed) {
    	moteurPilotage.setAngularSpeed(speed);
    }

    /**
     * Vérifie si le robot est en mouvement.
     *
     * @return true si le robot est en mouvement, false sinon.
     */
    public boolean isMoving() {
        return moteurPilotage.isMoving();
    }

     /**
     * Obtient l'orientation actuelle du robot.
     *
     * @return Orientation en degrés (0-360).
     */
    public double obtenirOrientation() {
        return orientation;
    }

    /**
     * Fait avancer le robot sur une distance donnée.
     *
     * @param distance Distance en centimètres à parcourir.
     */
    public void avancerDe(double distance) {
        moteurPilotage.travel(distance);
    }

    /**
     * Fait avancer le robot sur une distance donnée, avec possibilité d'action asynchrone.
     *
     * @param distance Distance en centimètres à parcourir.
     * @param b true si le déplacement doit être asynchrone.
     */
    public void avancerDe(double distance,boolean b) {
        moteurPilotage.travel(distance,b);
    }
    
     /**
     * Fait avancer le robot de manière continue (300 cm), avec option asynchrone.
     *
     * @param b true si l'action est asynchrone.
     */
    public void avancerContinu(boolean b) {
    	moteurPilotage.travel(300,b);
    }

    /**
     * Fait reculer le robot sur une distance donnée.
     *
     * @param distance Distance en centimètres à parcourir en marche arrière.
     */
    public void reculerDe(double distance) {
        moteurPilotage.travel(-distance);
    }

     /**
     * Arrête immédiatement les moteurs du robot.
     */
    public void stop() {
        moteurPilotage.stop();
    }

    
    /**
     * Fait pivoter le robot vers la gauche d'un angle donné.
     *
     * @param angleDeRotation Angle en degrés pour la rotation vers la gauche.
     */
    public void pivoterGauche(double angleDeRotation) {
        moteurPilotage.rotate(angleDeRotation);
        setOrientation(angleDeRotation);
    }
    
     /**
     * Fait pivoter le robot vers la droite d'un angle donné.
     *
     * @param angleDeRotation Angle en degrés pour la rotation vers la droite.
     */
    public void pivoterDroite(double angleDeRotation) {
        moteurPilotage.rotate(-angleDeRotation);
        setOrientation(-angleDeRotation);
    }
    
    /**
     * Effectue une rotation d'un angle donné, avec option d'exécuter une action en parallèle.
     *
     * @param angleDeRotation Angle de rotation en degrés.
     * @param asynch true si l'action est asynchrone.
     */
    public void rotate(double angleDeRotation, boolean asynch) {
    	moteurPilotage.rotate(angleDeRotation,asynch);
    }

    /**
     * Oriente le robot vers un angle cible donné.
     *
     * @param angleCible Angle cible en degrés (0-360).
     */
    public void allerVersOrientation(double angleCible) {
    	double angleDiff = (angleCible - orientation + 360) % 360;
        if (angleDiff > 180) {
            pivoterGauche(360 - angleDiff);
        } else {
            pivoterDroite(angleDiff);
        }
    }
    
    /**
     * Oriente le robot vers la ligne adverse (angle 0).
     */
    public void orienterVersLigneAdverse() {
        allerVersOrientation(0);
    }
}
