
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.port.SensorPort;

public class Pince {

	private static EV3TouchSensor bouton;  // Capteur connecté au port S2
	private EV3LargeRegulatedMotor motorPince;// Moteur connecté au port B

	// Constructeur pour initialiser le moteur de la pince
	public Pince() {
		Port s3 = LocalEV3.get().getPort("S3");
		motorPince = new EV3LargeRegulatedMotor(MotorPort.B); 
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
		if (verificationPalet()== 100) {
			fermer(1800);
			return true;
		}
		return false;
	}

	public void lacherPalet() {
		// Ouvre les pinces
		ouvrir(1800);
	}
	
	public void lacherApresLigneBlanche() {
		// Lache le palet une fois que la ligne détectée
		// Renvoie true s'il est lâché
		ColorSensorTest c = new ColorSensorTest(this);
	}

	public float verificationPalet() {
		//
		float a= retour();
		return a;
		//Va retourner 100 si c'est touché
		//Et 0 sinon rien n'est détecté
	}
	
	private float retour() {
		SampleProvider a = bouton.getTouchMode();
		float[] sample = new float[a.sampleSize()];
		a.fetchSample(sample,0);
		return (float) sample[0] *100;
	}
	
	public static void main(String[] args) {
		// Initialise l'application
		Pince app = new Pince();
		
		// Test		
		//app.recupererPalet();
		
		app.recupererPalet();
		
		//System.out.print(app.lacherApresLigneBlanche());
		

		//app.lacherPalet();
		// Ferme le moteur après utilisation
		app.motorPince.close();
	}

}
