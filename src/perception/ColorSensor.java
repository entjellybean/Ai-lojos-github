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

public class ColorSensor {
   EV3ColorSensor colorSensor;

   EV3LargeRegulatedMotor leftMotor;
   EV3LargeRegulatedMotor rightMotor;

   public static void main(String[] args) {
       new ColorSensor();
   }

   public ColorSensor() {}
	
   // Function to differentiate between gray or white
   private boolean detecteLigneBlanche() {
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
public void close() {
		colorSensor.close();
	}
}
