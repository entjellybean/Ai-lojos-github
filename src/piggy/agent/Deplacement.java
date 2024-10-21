
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Deplacement {
    private MovePilot pilot;  // Pilote pour gérer les déplacements
    private MoveScan capteurUltrason;  // Capteur pour éviter les obstacles
    private ColorSensorTest capteurCouleur;  // Capteur pour détecter la ligne blanche
    private Pince capteurContact;  // Capteur pour attraper et détecter les palets

    // Constructeur pour initialiser le déplacement et les capteurs
    public Deplacement() {
        // Configuration des roues et du châssis du robot
        Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, 0.056).offset(-0.06);  // Roue gauche
        Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 0.056).offset(0.06);  // Roue droite
        Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
        
        // Création du pilote pour le contrôle des déplacements
        pilot = new MovePilot(chassis);

        // Initialisation des capteurs
        capteurUltrason = new MoveScan();
        capteurCouleur = new ColorSensorTest();
        capteurContact = new Pince();
    }

    // Méthode pour faire avancer le robot d'une certaine distance
    public void avancer(double distance) {
        pilot.travel(distance);  // Avancer en centimètres
        Delay.msDelay(500);  // Petite pause pour éviter les collisions
    }

    // Méthode pour faire reculer le robot
    public void reculer(double distance) {
        pilot.travel(-distance);  // Reculer de la distance spécifiée
        Delay.msDelay(500);
    }

    // Méthode pour tourner à gauche d'un certain angle
    public void tournerGauche(double angle) {
        pilot.rotate(-angle);  // Tourner à gauche d'un angle donné en degrés
        Delay.msDelay(500);
    }

    // Méthode pour tourner à droite d'un certain angle
    public void tournerDroite(double angle) {
        pilot.rotate(angle);  // Tourner à droite d'un angle donné en degrés
        Delay.msDelay(500);
    }
    
    public void tourner(double angle) {
        pilot.rotate(angle);  // Tourner à gauche d'un angle donné en degrés
        Delay.msDelay(500);
    }

    // Détection des obstacles à l'aide du capteur ultrason
    public boolean detecterObstacle() {
        //
    	double distanceObstacle = capteurUltrason.getDistance();
        return distanceObstacle < 0.3;  //Renvoie si le robot est à moins de 30cm d'un obstacle
        // Si l'obstacle est à moins de 30 cm, le robot doit l'éviter
    }

    // Méthode pour éviter les obstacles détectés
    public void eviterObstacle() {
        if (detecterObstacle()) {
            System.out.println("Obstacle détecté, évidemment...");
            reculer(10);  // Reculer de 10 cm
            tournerDroite(45);  // Tourner à droite de 45 degrés pour éviter l'obstacle
            avancer(20);  // Avancer de 20 cm pour contourner l'obstacle
        }
        //mettre à jour l'attribut
    }

    // Méthode pour détecter un palet via le capteur de contact
    public boolean detecterPalet() {
        return capteurContact.estTouche();  // Retourne vrai si le palet est détecté via le capteur de contact
    }

    // Détection de la ligne blanche via le capteur de couleur
    public boolean detecterLigneBlanche() {
        String couleur = capteurCouleur.getCurrentColor();  // Obtenir la couleur actuelle du sol
        return couleur.equals("WHITE");  // Vérifier si c'est la ligne blanche
    }

    // Méthode pour orienter le robot vers la ligne blanche
    public void orienterVersLigneBlanche(int orientation) {
    	if (orientation != 90) {
    		int a = 90-orientation;
    		this.tourner(a);
    	}
    }

    // Méthode pour tester les mouvements de base et les capteurs
    public static void main(String[] args) {
        Deplacement robot = new Deplacement();

        // Tester le déplacement en avançant de 50 cm
        robot.avancer(50);
        
        // Tester l'évitement d'obstacles
        if (robot.detecterObstacle()) {
            robot.eviterObstacle();  // Éviter l'obstacle détecté
        }
        
        // Tester la détection de palet
        if (robot.detecterPalet()) {
            System.out.println("Palet détecté !");
        }

        // Tester l'orientation vers la ligne blanche
        robot.orienterVersLigneBlanche();  // Orienter le robot vers la ligne blanche
    }
}