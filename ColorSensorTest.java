package pack;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
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
       Port s3 = LocalEV3.get().getPort("S3");
       colorSensor = new EV3ColorSensor(s3);
      
       leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
       rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
       while (Button.ESCAPE.isUp()) {
           int currentDetectedColor = colorSensor.getColorID();
          
           switch (currentDetectedColor) {
               case Color.RED:
                   // Adjust direction to follow the red line
                   followLine();
                   break;
               case Color.GREEN:
                   // Turn 90 degrees when green is detected
                   turn90Degrees();
                   break;
              
               case Color.BLUE:
                   // Turn 90 degrees when green is detected
                   turn90Degrees();
                   break;
               case Color.GRAY:
                   // Turn 90 degrees when green is detected
                   turn90Degrees();
                   break;
               case Color.BLACK:
                   // Turn 90 degrees when green is detected
                   followLine();
                   break;
               default:
                   // Stop motors if no matching color
                   stopMotors();
                   break;
           }
           Delay.msDelay(250);  // Small delay between sensor readings
       }
      
       // Cleanup
       stopMotors();
       colorSensor.close();
   }
   // Function to follow the red line
   private void followLine() {
       // Assuming a simple line-following method:
       // If red is detected, go forward, otherwise make slight adjustments
       float[] colorSample = new float[colorSensor.sampleSize()];
       colorSensor.fetchSample(colorSample, 0);
       moveForward();
   /*     if (colorSample[0] > 1) { // If sensor detects a stronger red signal
           moveForward();
       } else {
           // Adjust to stay on the line (can modify this based on your robot's design)
          //slightTurnLeft();
       	slightTurnLeft();
       }*/
   }

  
   // Function to move forward
   private void moveForward() {
       leftMotor.setSpeed(300);  // Set motor speed
       rightMotor.setSpeed(300);
       leftMotor.forward();
       rightMotor.forward();
   }
   // Function for slight left turn (to stay on the red line)
/*   private void slightTurnLeft() {
       leftMotor.setSpeed(100);  // Slow down the left motor
       rightMotor.setSpeed(200);  // Right motor moves faster to adjust direction
       leftMotor.forward();
       rightMotor.forward();
   }*/
   // Function to turn 90 degrees (right turn)
   private void turn90Degrees() {
       leftMotor.setSpeed(200);  // Left motor moves forward
       rightMotor.setSpeed(200); // Right motor moves backward
       leftMotor.rotate(180, true); // Adjust the angle for a 90-degree turn
       rightMotor.rotate(-180, true);
       leftMotor.waitComplete();  // Wait until the rotation is complete
       rightMotor.waitComplete();
   }
   // Function to stop the motors
   private void stopMotors() {
       leftMotor.stop(true);
       rightMotor.stop(true);
   }
}

