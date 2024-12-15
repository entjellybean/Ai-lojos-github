package perception;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * La classe ColorSensor permet de gérer un capteur de couleur EV3. 
 * Elle est utilisée pour détecter des surfaces blanches en analysant 
 * les valeurs mesurées par le capteur.
 * @author Basak Unal
 */
public class ColorSensor {
	EV3ColorSensor colorSensor; // Capteur de couleur EV3

	public static void main(String[] args) {
		//new ColorSensor();
	}

	/**
        * Constructeur de la classe ColorSensor.
        * Initialise le capteur de couleur connecté à un port donné.
        *
        * @param s4 Port auquel le capteur de couleur EV3 est connecté.
        */
	public ColorSensor(Port s4) {
		colorSensor = new EV3ColorSensor(s4);
	}

	/**
        * Vérifie si une surface blanche est détectée.
        * La méthode utilise le mode "Red" du capteur pour mesurer la lumière réfléchie
        * et considère la surface blanche si les valeurs sont entre 40% et 80%.
        *
        * @return true si une surface blanche est détectée, false sinon.
        */
	public boolean detecteLigneBlanche() {
		SampleProvider lightProvider = colorSensor.getRedMode();
		float[] sample = new float[lightProvider.sampleSize()];
		lightProvider.fetchSample(sample, 0);
		float lightValue = sample[0] * 100; // Scale to percentage
		boolean b = false;
		for(int i=0; i<2;i++) {
			System.out.println("Light Value: " + lightValue);
		}

		if (lightValue > 40 && lightValue < 80) {
			b = true; //si la couleur est blanche
		} 
		return b;
	}

	/**
        * Ferme et termine l'utilisation du capteur de couleur.
        * Cette méthode doit être appelée pour libérer les ressources matérielles associées
        * au capteur une fois son utilisation terminée.
        */
	public void close() {
		colorSensor.close();
	}
}
