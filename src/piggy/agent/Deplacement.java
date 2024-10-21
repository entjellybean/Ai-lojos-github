
package Action;

import Perception.ColorSensor;
import Perception.UltraSonic;
import Perception.TouchSensor;
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class Deplacement {
    private MovePilot pilot;  // Pilote pour gérer les déplacements
    private UltraSonic capteurUltrason;  // Capteur à ultrasons pour détecter les obstacles
    private ColorSensor capteurCouleur;  // Capteur de couleur pour détecter la ligne blanche
    private TouchSensor capteurContact;  // Capteur de contact pour détecter les palets
    private Pince pince;  // Pince pour attraper et déposer les palets

    private boolean paletAttrape;  // Indique si un palet est actuellement attrapé

    // Constructeur pour initialiser les déplacements et les capteurs
    public Deplacement() {
        // Configuration des roues et du châssis du robot
        Wheel leftWheel = WheeledChassis.modelWheel(Motor.C, 0.056).offset(-0.06);  // Roue gauche
        Wheel rightWheel = WheeledChassis.modelWheel(Motor.A, 0.056).offset(0.06);  // Roue droite
        Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);

        // Création du pilote pour le contrôle des déplacements
        pilot = new MovePilot(chassis);

        // Initialisation des capteurs
        capteurUltrason = new UltraSonic();
        capteurCouleur = new ColorSensor();
        capteurContact = new TouchSensor();
        pince = new Pince();

        paletAttrape = false;  // Au départ, aucun palet n'est attrapé
    }

    // Méthode pour faire avancer le robot d'une certaine distance
    public void avancer(double distance) {
        while (distance > 0) {
            // Vérifie s'il y a un obstacle
            if (detecterObstacle()) {
                eviterObstacle();  // Si obstacle, l'éviter
            }

            // Vérifie s'il y a une ligne blanche
            if (detecterLigneBlanche()) {
                System.out.println("Ligne blanche détectée, arrêt...");
                arreter();  // Arrêt si la ligne blanche est détectée
                return;
            }

            // Si un palet est détecté, le ramasser
            if (detecterPalet()) {
                attraperPalet();
            }

            // Continue à avancer
            pilot.travel(10);  // Avance par segments de 10 cm
            distance -= 10;  // Réduit la distance restante
            Delay.msDelay(100);  // Petite pause pour laisser le robot réagir
        }
    }

    // Méthode pour reculer le robot
    public void reculer(double distance) {
        pilot.travel(-distance);  // Reculer de la distance spécifiée
    }

    // Méthode pour tourner à gauche d'un certain angle
    public void tournerGauche(double angle) {
        pilot.rotate(-angle);  // Tourner à gauche d'un angle donné en degrés
    }

    // Méthode pour tourner à droite d'un certain angle
    public void tournerDroite(double angle) {
        pilot.rotate(angle);  // Tourner à droite d'un angle donné en degrés
    }

    // Méthode pour tourner d'un certain angle
    public void tourner(double angle) {
        pilot.rotate(angle);  // Tourner d'un angle donné
    }

    // Détection des obstacles à l'aide du capteur ultrason
    public boolean detecterObstacle() {
        double distanceObstacle = capteurUltrason.getDistance();  // Mesurer la distance à l'obstacle
        return distanceObstacle < 0.3;  // Renvoie vrai si le robot est à moins de 30 cm d'un obstacle
    }

    // Méthode pour éviter les obstacles détectés
    public void eviterObstacle() {
        System.out.println("Obstacle détecté, évitement...");
        reculer(10);  // Reculer de 10 cm
        tournerDroite(45);  // Tourner à droite de 45 degrés pour éviter l'obstacle
        avancer(20);  // Avancer de 20 cm pour contourner l'obstacle
    }

    // Méthode pour détecter un palet via le capteur de contact
    public boolean detecterPalet() {
        return capteurContact.isPressed();  // Retourne vrai si un palet est détecté via le capteur de contact
    }

    // Méthode pour attraper un palet
    public void attraperPalet() {
        if (detecterPalet() && !paletAttrape) {
            pince.fermer();  // Fermer la pince pour attraper le palet
            paletAttrape = true;  // Indique qu'un palet est maintenant attrapé
            System.out.println("Palet attrapé !");
        }
    }

    // Méthode pour relâcher un palet
    public void relacherPalet() {
        if (paletAttrape) {
            pince.ouvrir();  // Ouvrir la pince pour relâcher le palet
            paletAttrape = false;  // Indique que le palet a été relâché
            System.out.println("Palet relâché !");
        }
    }

    // Détection de la ligne blanche via le capteur de couleur
    public boolean detecterLigneBlanche() {
        String couleur = capteurCouleur.getCurrentColor();  // Obtenir la couleur actuelle du sol
        return couleur.equals("WHITE");  // Vérifier si c'est la ligne blanche
    }

    // Arrête le robot
    public void arreter() {
        pilot.stop();  // Arrête tous les moteurs
    }

    // Méthode pour tester les mouvements de base et les capteurs
    public static void main(String[] args) {
        Deplacement robot = new Deplacement();

        // Avancer et éviter les obstacles
        robot.avancer(100);  // Avancer de 100 cm en détectant et évitant les obstacles
        
        // Reculer après avoir avancé
        robot.reculer(50);  // Reculer de 50 cm

        // Tourner et vérifier les obstacles
        robot.tournerDroite(90);  // Tourner à droite de 90 degrés
        robot.tournerGauche(90);  // Tourner à gauche de 90 degrés

        // Tester la pince en attrapant et relâchant un palet
        robot.attraperPalet();
        robot.reculer(20);  // Reculer avec le palet
        robot.relacherPalet();  // Relâcher le palet
    }
}}