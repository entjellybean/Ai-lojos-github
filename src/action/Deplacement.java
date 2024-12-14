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

public class Deplacement {

    
    private MovePilot moteurPilotage;
    
    private double orientation; 
    //permet de connaître l'orientation du robot par rapport à sa position de base

    
    public Deplacement(Port A,Port C) {
        // Configure le diamètre et l'écartement des roues en centimètres
    	EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(A);
    	EV3LargeRegulatedMotor motorC = new EV3LargeRegulatedMotor(C);
    	Wheel roueDroite = WheeledChassis.modelWheel(motorA, 5.6).offset(-6.850); // Configuration de la roue gauche
        Wheel roueGauche = WheeledChassis.modelWheel(motorC, 5.6).offset(6.850);   // Configuration de la roue droite
        Chassis baseRoues = new WheeledChassis(new Wheel[] {roueGauche, roueDroite}, WheeledChassis.TYPE_DIFFERENTIAL);
        moteurPilotage = new MovePilot(baseRoues);
        moteurPilotage.setLinearSpeed(27); // Vitesse linéaire définie à 10 cm/s
        moteurPilotage.setAngularSpeed(30); // Vitesse de rotation modérée
        orientation = 0;
    }

    public void setOrientation(double nouvelAngle) {
    	// Met à jour l'orientation du robot
    	// Garantie une orientation entre 0 et 360 degrés
        orientation = (nouvelAngle + 360) % 360; 
    }
    
    public void setAngularSpeed(int speed) {
    	//Ajuste la vitesse de rotation du robot
    	moteurPilotage.setAngularSpeed(speed);
    }
    
    public boolean isMoving() {
    	//renvoie vrai si le robot est en action
        return moteurPilotage.isMoving();
    }
    
    public double obtenirOrientation() {
    	//renvoie l'orientation actuelle du robot
        return orientation;
    }

    public void avancerDe(double distance) {
    	//Le robot avance de la distance
        moteurPilotage.travel(distance);
    }
    
    public void avancerDe(double distance,boolean b) {
    	//Le robot avance de la distance et fait une action en même temps
        moteurPilotage.travel(distance,b);
    }

    public void avancerContinu(boolean b) {
    	//Avance en sur 3 mètres, c'est à dire en continu car nous n'avons pas plus
    	//de longueur disponible sur le plateau. Si b = True, alors le robot peut
    	//faire une action en simultané
    	moteurPilotage.travel(300,b);
      
    }
    
    public void reculerDe(double distance) {
    	//Recule de la distance
        moteurPilotage.travel(-distance);
    }

    public void stop() {
    	//Ferme les moteurs
        moteurPilotage.stop();
    }

    public void pivoterGauche(double angleDeRotation) {
    	//Tourne à gauche de l'angleDeRotation
        moteurPilotage.rotate(angleDeRotation);
        setOrientation(angleDeRotation);
    }

    public void pivoterDroite(double angleDeRotation) {
    	//Tourne à droite de l'angleDeRotation
        moteurPilotage.rotate(-angleDeRotation);
        setOrientation(-angleDeRotation);
    }

    public void rotate(double angleDeRotation, boolean asynch) {
    	//Tourne et fait une autre action en même temps
    	moteurPilotage.rotate(angleDeRotation,asynch);
    }
    
    public void allerVersOrientation(double angleCible) {
    	//S'oriente vers un certain point (permet de choisir quel méthode pivoter utiliser
        double angleDiff = (angleCible - orientation + 360) % 360;
        if (angleDiff > 180) {
            pivoterGauche(360 - angleDiff);
        } else {
            pivoterDroite(angleDiff);
        }
    }

    public void orienterVersLigneAdverse() {
    	//Permet de s'orienter vers la position d'origine (donc 0), et donc d'être
    	//bien axé par rapport à la ligne blanche
        allerVersOrientation(0);
    }
}
