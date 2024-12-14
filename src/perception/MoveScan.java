package perception;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import action.Deplacement;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
//import lejos.robotics.navigation.Movedeplacement;
import lejos.utility.Delay;

/**
 * Classe pour gérer le mouvement du robot et effectuer des scans à l'aide d'un capteur ultrason.
 */
public class MoveScan {
    Deplacement deplacement; 
    private EV3UltrasonicSensor ultrasonicSensor; 

    /**
     * Constructeur pour créer un objet MoveScan avec un contrôleur de mouvement du robot.
     * 
     * @param mouvementRobot le contrôleur de mouvement du robot
     */
    public MoveScan(Deplacement mouvementRobot) {
        deplacement = mouvementRobot;
        ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
    }

    /**
     * Constructeur pour créer un objet MoveScan avec un contrôleur de mouvement du robot 
     * et un port spécifique pour le capteur.
     * 
     * @param mouvementRobot le contrôleur de mouvement du robot
     * @param s1 le port auquel le capteur ultrason est connecté
     */
    public MoveScan(Deplacement mouvementRobot, Port s1) {
        deplacement = mouvementRobot;
        ultrasonicSensor = new EV3UltrasonicSensor(s1);
    }

    /**
     * Obtient la distance mesurée par le capteur ultrason en centimètres.
     * 
     * @return la distance jusqu'à l'objet le plus proche en centimètres
     */
    public double getDistance() {
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];
        distanceMode.fetchSample(sample, 0);
        return (double) sample[0] * 100;
    }

    /**
     * Fait avancer le robot d'une distance spécifiée et s'arrête si un obstacle est détecté.
     * 
     * @param distance la distance à parcourir en centimètres
     * @return 0 si un obstacle est détecté, 1 sinon
     */
    public int forward(double distance) {
        deplacement.avancerDe(distance);
        double d = getDistance();
        while (deplacement.isMoving() && d >= 30) {
            d = getDistance();
        }
        deplacement.stop();
        return (d <= 30) ? 0 : 1;
    }

    /**
     * Compare deux tableaux pour déterminer s'ils sont égaux.
     * 
     * @param arr1 le premier tableau
     * @param arr2 le second tableau
     * @return true si les tableaux sont égaux, false sinon
     */
    public static boolean arraysEqual(double[] arr1, double[] arr2) {
        return Arrays.equals(arr1, arr2);
    }

    /**
     * Recherche et s'aligne sur des palettes détectées en fonction des discontinuités
     * dans les distances mesurées.
     * 
     * @return true si une palette est trouvée et alignée, false sinon
     */
    public boolean RechercheEtOrientation() {
        // Implémentation complète omise pour des raisons de brièveté
    }

    /**
     * Fait avancer le robot jusqu'à ce que la distance mesurée cesse de diminuer.
     */
    public void moveForward() {
        double distance = getDistance();
        deplacement.avancerDe(distance);
        boolean distanceDecrease = true;
        while (deplacement.isMoving() && distanceDecrease) {
            double newDistance = getDistance();
            if (distance - newDistance < 0) {
                distanceDecrease = false;
            }
        }
        deplacement.stop();
    }

    /**
     * Fait tourner le robot et collecte les mesures de distance.
     * 
     * @return une liste des distances mesurées pendant la rotation
     */
    public List<Double> rotateMeasure() {
        List<Double> distance = new ArrayList<>();
        deplacement.setAngularSpeed(100);
        deplacement.rotate(330, true);
        while (deplacement.isMoving()) {
            double d = this.getDistance();
            distance.add(Double.isInfinite(d) ? 1000.0 : d);
        }
        deplacement.stop();
        return distance;
    }

    /**
     * Détecte la première discontinuité dans une liste de distances basée sur un seuil.
     * 
     * @param distances la liste des distances
     * @param threshold le seuil pour détecter les discontinuités
     * @return un tableau contenant les index de début et de fin de la discontinuité
     */
    public int[] detectPallet(List<Double> distances, double threshold) {
        // Implémentation complète omise pour des raisons de brièveté
    }

    /**
     * Trouve toutes les discontinuités dans une liste de distances basées sur un seuil.
     * 
     * @param distances la liste des distances
     * @param threshold le seuil pour détecter les discontinuités
     * @return une liste de tableaux, chaque tableau contenant deux index et la distance au premier index
     */
    public List<double[]> findAllDiscontinuities(List<Double> distances, double threshold) {
        // Implémentation complète omise pour des raisons de brièveté
    }

    /**
     * Trouve la discontinuité avec la plus petite distance dans une liste de discontinuités.
     * 
     * @param discontinuities la liste des discontinuités
     * @return un tableau contenant deux index et la plus petite distance
     */
    public double[] findSmallestDiscontinuity(List<double[]> discontinuities) {
        // Implémentation complète omise pour des raisons de brièveté
    }

    /**
     * Calcule l'angle moyen entre deux index en fonction du nombre total de mesures.
     * 
     * @param idx un tableau contenant deux index
     * @param size le nombre total de mesures
     * @return l'angle moyen en degrés
     */
    public static double calc_angle(double[] idx, int size) {
        // Implémentation complète omise pour des raisons de brièveté
    }

    /**
     * Vérifie si la plus petite discontinuité correspond à une palette.
     * 
     * @param smallestDiscontinuity la plus petite discontinuité
     * @param distances la liste des distances
     * @param stabilityThreshold le seuil de stabilité
     * @param lengthThreshold le seuil de longueur
     * @return true si la discontinuité représente une palette, false sinon
     */
    public boolean isPalet(double[] smallestDiscontinuity, List<Double> distances,
            double stabilityThreshold, double lengthThreshold) {
        // Implémentation complète omise pour des raisons de brièveté
    }

    /**
     * Ferme le capteur ultrason pour libérer les ressources.
     */
    public void close() {
        ultrasonicSensor.close();
    }
}
