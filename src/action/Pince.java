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

	private EV3TouchSensor bouton;
	//le bouton était initialement prévu pour permettre de savoir si il y avait un palet
	//mais comme il était pas assez sensible, nous ne l'avons pas utilisé dans nos classes au final
	
	EV3MediumRegulatedMotor motorPince;
	
	// Constructeur pour initialiser le moteur de la pince
	public Pince(Port B,Port s3) {
		
		motorPince = new EV3MediumRegulatedMotor(B); 
		bouton=new EV3TouchSensor(s3);
		
	}

	public void ouvrir(int a) {
		// Ouvre la pince
		motorPince.rotate(a);
	}

	public void fermer(int a) {
		// Fermer la pince
		motorPince.rotate(-a);
	}

	public void stop() {
		// Ferme le moteur de la pince
		motorPince.stop();
	}

	public boolean recupererPalet() {
		// Ferme les pinces et renvoit vrai si le palet est récupéré
		fermer(1500);
		return true;
	}


	public void lacherPalet() {
		// Ouvre les pinces
		ouvrir(1500);
	}
	
	public void close() {
		//ferme le moteur de la pince
		this.motorPince.close();
	}

}
