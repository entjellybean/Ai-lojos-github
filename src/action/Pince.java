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

public class Pince {

	private EV3TouchSensor bouton;  // Capteur connecté au port S3
	//private EV3LargeRegulatedMotor motorPince;// Moteur connecté au port B

	EV3MediumRegulatedMotor motorPince;
	
	// Constructeur pour initialiser le moteur de la pince
	public Pince(Port B,Port s3) {
		
		motorPince = new EV3MediumRegulatedMotor(B); 
		bouton=new EV3TouchSensor(s3);
		
	}

	public void ouvrir(int a) {
		// Ouvre la pince
		motorPince.rotate(a);
		
		//2000 pour ouvrir complètement
	}

	public void fermer(int a) {
		// Fermer la pince
		motorPince.rotate(-a); 
		
		//2000 pour fermer complètement
	}

	public void stop() {//Je sais pas comment le tester
		// Arrête le moteur de la pince
		motorPince.stop();
	}

	public boolean recupererPalet() {
		// Détecter le palet et ferme les pinces
		// Renvoit vrai si le palet est récupéré
		/*if (verificationPalet()== 100) {
			fermer(1800);
			return true;
		}
		return false;*/
		fermer(1500); //avant 1800
		return true;
	}

	public void lacherPalet() {
		// Ouvre les pinces
		ouvrir(1500); //avant 1800
	}
	
	public void close() {
		this.motorPince.close();
	}

}
