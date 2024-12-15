package perception;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import action.Deplacement;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * Classe pour gérer le mouvement du robot et effectuer des scans à l'aide d'un capteur ultrason.
 * @author Narta Neziraj
 */
public class MoveScan {
    Deplacement deplacement; // Contrôle du robot
    private EV3UltrasonicSensor ultrasonicSensor; // Capteur de distance

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
        // Collecte des mesures de distance en effectuant une rotation
        List<Double> distance = rotateMeasure();

        // Détecte les discontinuités dans les mesures de distance avec un seuil de 2
        List<double[]> res = findAllDiscontinuities(distance, 2);

        // Trouve la plus petite discontinuité dans la liste des discontinuités détectées
        double[] resFinal = findSmallestDiscontinuity(res);

        // Parcours la liste des discontinuités pour retirer celle qui correspond à la plus petite discontinuité
        for (int i = 0; i < res.size(); i++) {
            // Si la discontinuité actuelle est égale à la plus petite discontinuité trouvée
            if (arraysEqual(res.get(i), resFinal)) {
                // Retirer cette discontinuité de la liste pour éviter les doublons
                res.remove(i);
                break; // Sortir de la boucle dès que la discontinuité est retirée
            }
        }

        // Trouver la nouvelle plus petite discontinuité après suppression de la précédente
        double[] res_final2 = findSmallestDiscontinuity(res);

        // Vérifier si la première discontinuité est une palette en utilisant la fonction isPalet
        boolean b1 = isPalet(resFinal, distance, 3, 5);
       
        // Vérifier si la deuxième discontinuité est une palette
        boolean b2 = isPalet(res_final2, distance, 3, 5);
      

        // Si la première discontinuité est une palette
        if (b1) {
            // Afficher les discontinuités restantes pour diagnostic
            for (double[] arr : res) {
                System.out.println(Arrays.toString(arr));
            }
            // Afficher la première discontinuité pour diagnostic
            System.out.println(Arrays.toString(resFinal));

            // Calculer l'angle de rotation nécessaire pour s'aligner sur la palette
            double angle = calc_angle(resFinal, distance.size());
            System.out.println(angle);

            // Calculer la distance cible à parcourir pour atteindre la palette
            double targetDistance = resFinal[2];

            // Définir une tolérance de 5 cm pour le déplacement
            double tolerance = 5; // 3 cm de tolérance



          
            if (angle > 180) {
                // Pivoter à droite si l'angle est supérieur à 180°
                deplacement.pivoterDroite(360 - angle);
                
            } else {
                // Pivoter à gauche sinon
                deplacement.pivoterGauche(angle);
            }

          

           
            deplacement.avancerDe(targetDistance - 15);
            return true; // Retourner true car la palette a été trouvée et l'alignement est effectué

        } else if (b2) {
           
            // Calculer l'angle de rotation nécessaire pour s'aligner sur la palette
            double angle = calc_angle(resFinal, distance.size());
           
            // Calculer la distance cible à parcourir pour atteindre la palette
            double targetDistance = resFinal[2];

            // Définir une tolérance de 5 cm pour le déplacement
            double tolerance = 5;

           

            // Ajuster l'angle en fonction de la distance de la palette pour améliorer la précision
            if (angle > 180) {
                // Pivoter à droite si l'angle est supérieur à 180°
                deplacement.pivoterDroite(360 - angle);
               
            } else {
                // Pivoter à gauche sinon
                deplacement.pivoterGauche(angle);
            }

           

            // Avancer vers la palette, en ajustant la distance cible (en retirant 15 cm pour la tolérance)
            deplacement.avancerDe(targetDistance - 15);
            return true; // Retourner true car la palette a été trouvée et l'alignement est effectué

        } else {
            // Si aucune palette n'a été trouvée dans les deux premières discontinuités
            // Faire pivoter le robot de 90° pour explorer une nouvelle direction
            deplacement.pivoterDroite(90);
            // Avancer de 30 cm pour se déplacer un peu
            deplacement.avancerDe(30);

            return false; // Retourner false car aucune palette n'a été trouvée
        }
        // Fin de la méthode, la boucle s'arrête une fois que le robot est aligné
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
     * Trouve toutes les discontinuités dans une liste de distances basées sur un seuil.
     * 
     * @param distances la liste des distances
     * @param threshold le seuil pour détecter les discontinuités
     * @return une liste de tableaux, chaque tableau contenant deux index et la distance au premier index
     */
    public List<double[]> findAllDiscontinuities(List<Double> distances, double threshold) {
        List<double[]> discontinuities = new ArrayList<>();
        for (int i = 0; i < distances.size() - 1; i++) {
            if (Math.abs(distances.get(i) - distances.get(i + 1)) > threshold) {
                discontinuities.add(new double[]{i, i + 1, distances.get(i)});
            }
        }
        return discontinuities;
    }

    /**
     * Trouve la discontinuité avec la plus petite distance dans une liste de discontinuités.
     * 
     * @param discontinuities la liste des discontinuités
     * @return un tableau contenant deux index et la plus petite distance
     */
    public double[] findSmallestDiscontinuity(List<double[]> discontinuities) {
        double[] smallest = discontinuities.get(0);
        for (double[] dis : discontinuities) {
            if (dis[2] < smallest[2]) {
                smallest = dis;
            }
        }
        return smallest;
    }

    /**
     * Calcule l'angle moyen entre deux index en fonction du nombre total de mesures.
     * 
     * @param idx un tableau contenant deux index
     * @param size le nombre total de mesures
     * @return l'angle moyen en degrés
     */
    public static double calc_angle(double[] idx, int size) {
        double startAngle = (360.0 / size) * idx[0];
        double endAngle = (360.0 / size) * idx[1];
        return (startAngle + endAngle) / 2;
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
        int startIdx = (int) smallestDiscontinuity[0];
        int endIdx = (int) smallestDiscontinuity[1];
        double length = distances.get(endIdx) - distances.get(startIdx);
        double stability = Math.abs(distances.get(startIdx) - distances.get(endIdx));

        return length > lengthThreshold && stability < stabilityThreshold;
    }

    /**
     * Ferme le capteur ultrason pour libérer les ressources.
     */
    public void close() {
        ultrasonicSensor.close();
    }
}
