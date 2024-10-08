package pack;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ColorSensorTest {
   EV3ColorSensor colorSensor;

   EV3LargeRegulatedMotor leftMotor;
   EV3LargeRegulatedMotor rightMotor;

   public static void main(String[] args) {
       new ColorSensorTest();
   }

   public ColorSensorTest() {
       // Sensor and motor initialization
       Port s4 = LocalEV3.get().getPort("S4");
       colorSensor = new EV3ColorSensor(s4);

       leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
       rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
       while (Button.ESCAPE.isUp()) {
           int currentDetectedColor = colorSensor.getColorID();
           System.out.print(currentDetectedColor);

           switch (currentDetectedColor) {
               case Color.RED:
                   // Adjust direction to follow the red line
                   followLine();
                   break;
               case Color.GREEN:
                   turn90Degrees();
                   break;
               case Color.BLUE:
                   turn90Degrees();
                   break;
               case Color.BLACK:
            	   grayOrWhite();
                  break;
               case Color.WHITE:
                   grayOrWhite(); // Handle white or gray detection
                   break;
               default:
                   // Stop motors if no matching color
                   moveForward();
                   break;
           }
           Delay.msDelay(300);  // Small delay between sensor readings
       }

       // Cleanup
       stopMotors();
       colorSensor.close();
   }

   // Function to follow the red line
   private void followLine() {
       moveForward();
   }

   // Function to move forward
   private void moveForward() {
       leftMotor.setSpeed(250);  // Set motor speed
       rightMotor.setSpeed(250);
       leftMotor.forward();
       rightMotor.forward();
   }

   // Function to turn 90 degrees (right turn)
   private void turn90Degrees() {
       leftMotor.setSpeed(200);  // Left motor moves forward
       rightMotor.setSpeed(200); // Right motor moves backward
       leftMotor.rotate(180, true); // Adjust the angle for a 90-degree turn
       rightMotor.rotate(-180, true);
       leftMotor.waitComplete();  // Wait until the rotation is complete
       rightMotor.waitComplete();
       moveForward();
   }

   // Function to differentiate between gray or white
   private void grayOrWhite() {
       SampleProvider lightProvider = colorSensor.getRedMode();
       float[] sample = new float[lightProvider.sampleSize()];
       lightProvider.fetchSample(sample, 0);
       float lightValue = sample[0] * 100; // Scale to percentage
       for(int i=0; i<2;i++) {
    	   System.out.println("Light Value: " + lightValue);
	   }
      
       if (lightValue > 40 && lightValue < 80) {
    	   
         stopMotors();
         Delay.msDelay(250);
         turn90Degrees();
         turn90Degrees();
       } else {
    	   moveForward();
       }
   }

   // Function to stop the motors
   private void stopMotors() {
       leftMotor.stop(true);
       rightMotor.stop(true);
   }
}
