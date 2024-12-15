package action;

import lejos.hardware.ev3.LocalEV3;


import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.hardware.port.SensorPort;
import lejos.hardware.port.TachoMotorPort;

/**
 * La classe Pince permet de contrôler les mouvements d'une pince  
 * sur un robot EV3. 
 * Elle permet d'ouvrir, fermer, récupérer et lâcher des palets.
 * @author Zoe Laget Thomas
 */
public class Pince {

	private EV3TouchSensor bouton; // Capteur tactile 
	EV3MediumRegulatedMotor motorPince; // Moteur contrôlant la pince
	
	/**
        * Constructeur de la classe Pince.
        * Initialise le moteur de la pince et le capteur tactile.
        *
        * @param B Port pour le moteur de la pince.
        * @param s3 Port pour le capteur tactile.
        */
	public Pince(Port B,Port s3) {
		
		motorPince = new EV3MediumRegulatedMotor(B); 
		bouton=new EV3TouchSensor(s3);
		
	}

	/**
        * Ouvre la pince 
        *
        * @param a Angle de rotation en degrés pour ouvrir la pince.
        */
	public void ouvrir(int a) {
		motorPince.rotate(a);
	}
	
	/**
        * Ferme la pince en tournant le moteur dans la direction opposée.
        *
        * @param a Angle de rotation en degrés pour fermer la pince.
        */
	public void fermer(int a) {
		motorPince.rotate(-a);
	}

        /**
        * Arrête le moteur de la pince immédiatement.
        */
	public void stop() {
		motorPince.stop();
	}

        /**
        * Ferme la pince pour récupérer un palet.
        * Cette méthode ferme la pince et renvoie true pour indiquer que 
        * le palet est supposé être récupéré.
        *
        * @return true si le palet est récupéré.
        */
	public boolean recupererPalet() {
		fermer(1500);
		return true;
	}

	/**
        * Ouvre la pince pour relâcher un palet.
        */
	public void lacherPalet() {
		ouvrir(1500);
	}

	/**
        * Arrête et termine l'utilisation du moteur de la pince.
        */
	public void close() {
		//ferme le moteur de la pince
		this.motorPince.close();
	}

}
