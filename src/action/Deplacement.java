package action;

import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

/**
 * La classe Deplacement permet de contrôler les déplacements de base du robot.
 */
public class Deplacement {

	/**
	 * Le MovePilot gère les mouvements du robot.
	 */
	private MovePilot moteurPilotage;

	/**
	 * Orientation actuelle du robot, en degrés. 0 représente le nord, aligné avec l’adversaire.
	 */
	private double orientation = 0;

	/**
	 * Constructeur pour initialiser le système de déplacement.
	 * Configure les roues et initialise le MovePilot.
	 */
	public Deplacement() {
		// Configure le diamètre et l'écartement des roues en centimètres
		Wheel roueGauche = WheeledChassis.modelWheel(Motor.A, 5.6).offset(-6.0);  // Configuration de la roue gauche
		Wheel roueDroite = WheeledChassis.modelWheel(Motor.C, 5.6).offset(6.0);   // Configuration de la roue droite
		Chassis baseRoues = new WheeledChassis(new Wheel[] {roueGauche, roueDroite}, WheeledChassis.TYPE_DIFFERENTIAL);
		moteurPilotage = new MovePilot(baseRoues);
		moteurPilotage.setAngularSpeed(50); // Vitesse de rotation modérée
	}

	/**
	 * Met à jour l'orientation actuelle du robot.
	 * @param nouvelAngle Angle en degrés
	 */
	public void setOrientation(double nouvelAngle) {
		this.orientation = nouvelAngle;
	}

	/**
	 * Récupère l'instance de MovePilot.
	 * @return moteurPilotage
	 */
	public MovePilot obtenirPilot() {
		return moteurPilotage;
	} 

	/**
	 * Renvoie l'orientation actuelle du robot.
	 * @return orientation
	 */
	public double obtenirOrientation() {
		return orientation;
	}

	/**
	 * Fait avancer le robot sur une distance spécifique en centimètres.
	 * @param distance Centimètres à parcourir
	 */
	public void avancerDe(double distance) {
		moteurPilotage.travel(distance);
	}

	/**
	 * Lance un déplacement en avant sans limite de distance (le robot avance jusqu'à l'arrêt).
	 */
	public void avancerContinu() {
		moteurPilotage.forward();
	}

	/**
	 * Recule le robot d'une distance spécifiée en centimètres.
	 * @param distance Centimètres à parcourir en arrière
	 */
	public void reculerDe(double distance) {
		moteurPilotage.travel(-distance);
	}

	/**
	 * Recule sans limite de distance (le robot recule jusqu'à l'arrêt).
	 */
	public void reculerContinu() {
		moteurPilotage.backward();
	}

	/**
	 * Stoppe le robot instantanément.
	 */
	public void arretImmediat() {
		moteurPilotage.stop();
	}

	/**
	 * Effectue une rotation vers la gauche d'un certain angle.
	 * Ajuste l'orientation du robot en conséquence.
	 * @param angleDeRotation Angle à tourner en degrés
	 */
	public void pivoterGauche(double angleDeRotation) {
		moteurPilotage.rotate(-angleDeRotation);
		ajusterOrientation(-angleDeRotation);
	}

	/**
	 * Effectue une rotation vers la droite d'un certain angle.
	 * Ajuste l'orientation du robot en conséquence.
	 * @param angleDeRotation Angle à tourner en degrés
	 */
	public void pivoterDroite(double angleDeRotation) {
		moteurPilotage.rotate(angleDeRotation);
		ajusterOrientation(angleDeRotation);
	}

	/**
	 * Ajuste l'orientation actuelle en fonction de l'angle de rotation donné.
	 * @param angleDeRotation L'angle ajouté ou soustrait à l'orientation actuelle
	 */
	private void ajusterOrientation(double angleDeRotation) {
		orientation += angleDeRotation;
		if (orientation >= 360) {
			orientation -= 360;
		} else if (orientation < 0) {
			orientation += 360;
		}
	}

	/**
	 * Oriente le robot vers sa position de départ, dirigée vers l’en-but adverse.
	 */
	public void retourInitial() {
		if (orientation < 180) {
			pivoterGauche(orientation);
		} else {
			pivoterDroite(360 - orientation);
		}
	}

	/**
	 * Oriente le robot vers une position spécifique en degrés.
	 * @param angleCible Angle souhaité
	 */
	public void allerVersOrientation(double angleCible) {
		double angleDiff = angleCible - orientation;
		if (angleDiff > 0) {
			pivoterDroite(angleDiff);
		} else {
			pivoterGauche(-angleDiff);
		}
	}
	
	public void avancerVers1erPalet(Deplacement robot) {
		robot.avancerDe(100); //vérifier longueur
	}
	
	public static void main(String[] args) {
		// Crée une instance de la classe Deplacement pour contrôler le robot
		Deplacement robot = new Deplacement();

		// Avancer de 20 cm
		/*System.out.println("Avancer de 20 cm");
		robot.avancerDe(20);

		// Pause pour observer le mouvement
		//  attendre(2000);

		// Reculer de 20 cm
		System.out.println("Reculer de 20 cm");
		robot.reculerDe(20);

		// Pause pour observer le mouvement
		//  attendre(2000);

		// Pivoter à gauche de 90 degrés
		System.out.println("Pivoter à gauche de 90 degrés");
		robot.pivoterGauche(90);

		// Pause pour observer le mouvement
		//   attendre(2000);

		// Pivoter à droite de 90 degrés
		System.out.println("Pivoter à droite de 90 degrés");
		robot.pivoterDroite(90);*/

		// Avancer en continu (le robot avancera jusqu'à ce qu'il soit arrêté)
		System.out.println("Avancer en continu");
		robot.avancerContinu();

		System.out.println("Orientation");
		robot.allerVersOrientation(90);
		
		
		robot.retourInitial();
		
		robot.ajusterOrientation(90);
	}
}
