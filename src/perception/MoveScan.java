import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class MoveScan {
	public MovePilot pilot; // control the robot 
	private EV3UltrasonicSensor ultrasonicSensor; //the distance sensor
	//private static final int TOTAL_Degrees=4;
	//private double [] distances=new double [TOTAL_Degrees];
	double wheelDiameter;// array to hold the measured distances
	
	
		private float currentDistance;
		
		
		private float lastDistance;

	public static void main(String[] args) {
		MoveScan robot = new MoveScan();
        //List<Double> distance = robot.rotateMeasure();
       
       // double x_index=robot.detectDiscontinuity(distance,40);
        //int[] inde=robot.detectPallet(distance, 40);
        //System.out.println("first index : "+inde[0]);
        //System.out.println("second index : "+inde[1]);
        //List<double[]> res=robot.findAllDiscontinuities(distance, 40);
        //double [] resFinal =robot.findSmallestDiscontinuity(res);
        
        /*for (double[] arr : res) {
            System.out.println(Arrays.toString(arr));
        }
        System.out.println(Arrays.toString(resFinal));
        double angle=robot.calc_angle(resFinal, distance.size());
        //System.out.println(angle+10);
        robot.pilot.rotate(angle, false);
        double current_dist=robot.getDistance();
        System.out.println(current_dist);*/
        
        //String what=robot.analyzeDiscontinuity(resFinal,distance,10,15,2);
        //System.out.println(what);
        //double gjatesi=robot.calculateObjectLength(distance, resFinal[0],resFinal[1]);
        //System.out.println(gjatesi);
        
       
       /* int j = 0;
       for (Double d : distance) {
			 System.out.println(j + ":" + d);
			 j++;
		 
	    }*/
        
        robot.moveWhile();
		



	}
	
	public MoveScan() {
		
		
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, 0.056).offset(-0.06);  
		//the wheel diameter 56 mm, The offset specifies the position of 
		//each wheel relative to the robot’s center.
	    Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 0.056).offset(0.06);  // Roue droite
	    Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel, rightWheel},WheeledChassis.TYPE_DIFFERENTIAL);
 
	    pilot = new MovePilot(chassis);
	    ultrasonicSensor=new EV3UltrasonicSensor(SensorPort.S2);

	}
	
	 private double getDistance() {
	        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
	        float[] sample = new float[distanceMode.sampleSize()];
	        distanceMode.fetchSample(sample, 0);
	        return (double) sample[0] * 100; // Convert to centimeters
	    }
	// Retourne la distance actuelle
		public float getCurrentDistance() {
			return currentDistance;
		}
		
		// Retourne la derni�re distance mesur�e
		public float getLastDistance() {
			return lastDistance;
		}
		
		// Modifie la distance actuelle
		public void setCurrentDistance(float currentDistance) {
			this.currentDistance = currentDistance;
		}
		
		// Modifie la derni�re distance mesur�e
		public void setLastDistance(float lastDistance) {
			this.lastDistance = lastDistance;
		}
		
		public void  moveWhile() {
			List<Double> distance = rotateMeasure();
		       
		        List<double[]> res=findAllDiscontinuities(distance, 40);
		        double [] resFinal =findSmallestDiscontinuity(res);
		        
		        for (double[] arr : res) {
		            System.out.println(Arrays.toString(arr));
		        }
		        System.out.println(Arrays.toString(resFinal));
		        double angle=calc_angle(resFinal, distance.size());
		        //System.out.println(angle+10);
		        pilot.rotate(angle, false);
		        double current_dist=getDistance();
		        System.out.println(current_dist);
		        while(current_dist!=resFinal[2]) {
		        	pilot.rotate(1,true);
		        }
		        
		        
			
		}
		
	 
	 public List<Double> rotateMeasure() {
		 List<Double> distance = new ArrayList<Double>();
		 pilot.setAngularSpeed(100);
		 pilot.rotate(370, true);
		 while(pilot.isMoving()) {
			 double d=this.getDistance();
			 if(Double.isInfinite(d)) {
				 distance.add(1000.0);
			 }
			 else {
				 distance.add(d);
				 
			 }
			 System.out.println(distance.size()-1 + " " + distance.get(distance.size()-1));
			 
			 //Delay.msDelay(10);
			 
		 }
		 pilot.stop();
		return distance;
	 }
	 
	 
	
	 //return the indexes of the first discontinuity
	 public int[] detectPallet(List<Double> distances, double threshold) {
		 int count=0;
		 int[] indexes=new int[2];
		 indexes[0] = -1;
		 indexes[1] = -1;
		 
		 for (int i = 0; i < distances.size() - 1; i++) {   	
			 double difference = distances.get(i + 1) - distances.get(i);
		     if (difference<0 && Math.abs(difference)> threshold) {  
		    	 indexes[0] = i+1;
		    	 break;
		    	 //System.out.println("first index : "+indexes[0]);
		    }
		 }
		 if (indexes[0] == -1) return indexes; 
		
		 for (int i = indexes[0]; i < distances.size() - 1; i++) {   	
			 double difference = distances.get(i + 1) - distances.get(i);
			 if (difference>0 && difference>threshold) {
		        	indexes[1]=i+1;
		        	break;
		        	//System.out.println("second index : "+indexes[1]);
			 }
		 }
		 return indexes;
	 }
	 
	 //return a list of lists with 2 indexes of all the discontinuities
	 public List<double[]> findAllDiscontinuities(List<Double> distances, double threshold) {
		    List<double[]> discontinuities = new ArrayList<>();
		    int firstIndex = -1;

		    for (int i = 0; i < distances.size() - 1; i++) {
		        double difference = distances.get(i + 1) - distances.get(i);

		        // Detect the first negative change above the threshold
		        if (firstIndex == -1 && difference < 0 && Math.abs(difference) > threshold) {
		            firstIndex = i + 1;
		        }
		        // After finding a negative change, look for a subsequent positive change
		        else if (firstIndex != -1 && difference > 0 && difference > threshold) {
		            int secondIndex = i ;
		            double distanceAtFirstIndex = distances.get(firstIndex);
		            discontinuities.add(new double[]{firstIndex, secondIndex, distanceAtFirstIndex});
		            firstIndex = -1;  // Reset to look for the next pallet
		        }
		    }

		    return discontinuities;
		}
	 
	 //we pick the smallest distance discontinuity
		public double[] findSmallestDiscontinuity(List<double[]> discontinuities) {
		    if (discontinuities.isEmpty()) {
		        return new double[]{-1, -1, -1}; // Or another indication that no discontinuity was found
		    }

		    // Initialize with the first discontinuity
		    double[] smallestDiscontinuity = discontinuities.get(0);
		    for (double[] discontinuity : discontinuities) {
		        if (discontinuity[2] < smallestDiscontinuity[2]) {
		            smallestDiscontinuity = discontinuity;
		        }
		    }

		    // Return only the first and second index with the smallest distance at the first index
		    return new double[]{smallestDiscontinuity[0], smallestDiscontinuity[1],smallestDiscontinuity[2]};
		}

	 
	 //return the average angle of 2 indexes
	 public static double calc_angle(double [] idx,int size) {
		 double min=idx[0];
		 double max=idx[1];
		 double angle_min=0;
		 double angle_max=0;
		 if (max != -1 && min!= -1) {
		        double x =  370.0 / size;  
		        angle_min = min * x;
		         angle_max = max * x;
		        }
		 return (angle_min+angle_max)/2;
		        
		
	 }
	
	 public double calculateObjectLength(List<Double> distances, double first, double second) {
		    // Calculate the average distance to the object over the observed range
		 	int firstIndex=(int) first;
		 	int secondIndex=(int)second;
		    double averageDistance = (distances.get(firstIndex) + distances.get(secondIndex)) / 2;

		    // Calculate the angle in radians based on the indices and total rotation
		    double angleInDegrees = Math.abs(secondIndex - firstIndex) * (370.0 / distances.size());
		    double angleInRadians = Math.toRadians(angleInDegrees);

		    // Calculate the arc length as the object's length
		    double objectLength = averageDistance * angleInRadians;
		    return objectLength;
		}
	 
	 public boolean isStableBetween(List<Double> distances, int firstIndex, int secondIndex, double stabilityThreshold) {
		    for (int i = firstIndex; i < secondIndex; i++) {
		        double diff = Math.abs(distances.get(i + 1) - distances.get(i));
		        if (diff > stabilityThreshold) {
		            return false;  // Not stable
		        }
		    }
		    return true;  // Stable range
		}
	 public String analyzeDiscontinuity(double[] smallestDiscontinuity, List<Double> distances, double stabilityThreshold, double distanceThreshold, double decreaseThreshold) {
		    int firstIndex = (int) smallestDiscontinuity[0];
		    int secondIndex = (int) smallestDiscontinuity[1];
		    
		    // Check if the readings are stable between the discontinuity points
		    if (isStableBetween(distances, firstIndex, secondIndex, stabilityThreshold)) {
		        // Calculate the estimated length of the object
		        double estimatedDistance = calculateObjectLength(distances, firstIndex, secondIndex);

		        if (estimatedDistance > distanceThreshold) {
		            return "Wall detected";
		        } else {
		            return "Pallet detected";
		        }
		    } 
		    // If not stable, check if the object is moving toward us
		    else if (isObjectMovingToward(distances, firstIndex, secondIndex)) {
		        return "Object moving toward detected";
		    } else {
		        return "Inconsistent readings; no wall or pallet detected";
		    }
		}


	 
	 
	 
	 public boolean isObjectMovingToward(List<Double> distances, int firstIndex, int secondIndex) {
		    // Ensure there are enough points to evaluate movement
		    if (secondIndex <= firstIndex + 1) {
		        return false;
		    }

		    // Check if each distance between firstIndex and secondIndex is decreasing
		    for (int i = firstIndex; i < secondIndex - 1; i++) {
		        double currentDistance = distances.get(i);
		        double nextDistance = distances.get(i + 1);
		        
		        
		        if ( nextDistance < currentDistance) {
		            return false;
		        }
		    }
		    
		    // If we complete the loop with consistent decreases, return true
		    return true;
		}

	 
	 
	


}
