package perception;


import java.util.ArrayList;


import java.util.Arrays;
import java.util.List;

import action.Deplacement;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
//import lejos.robotics.navigation.Movedeplacement;
import lejos.utility.Delay;

public class MoveScan {
	//public Deplacement deplacement;
	Deplacement deplacement;// control the robot
	private EV3UltrasonicSensor ultrasonicSensor; //the distance sensor

	double wheelDiameter;// array to hold the measured distances


	private float currentDistance;


	private float lastDistance;

	public static void main(String[] args) {
		//MoveScan robot = new MoveScan();
		//int rezultat=robot.forward(70);
		//System.out.println(rezultat);

		/*List<Double> distance = robot.rotateMeasure();

        //List<double[]> x_index=robot.findAllDiscontinuities(distance,40);

        int[] inde=robot.detectPallet(distance, 40);
        System.out.println("first index : "+inde[0]);
        System.out.println("second index : "+inde[1]);
        List<double[]> res=robot.findAllDiscontinuities(distance, 40);
        double [] resFinal =robot.findSmallestDiscontinuity(res);

        for (double[] arr : res) {
            System.out.println(Arrays.toString(arr));
        }
        System.out.println(Arrays.toString(resFinal));
        double angle=robot.calc_angle(resFinal, distance.size());
        //System.out.println(angle+10);
        //robot.deplacement.rotate(angle, false);
        //double current_dist=robot.getDistance();
        //System.out.println(current_dist);

        String what=robot.analyzeDiscontinuity(resFinal,distance,5,15);
        System.out.println(what);
        double gjatesi=robot.calculateObjectLength(distance, resFinal[0],resFinal[1]);
        System.out.println(gjatesi);


       /* int j = 0;
       for (Double d : distance) {
System.out.println(j + ":" + d);
j++;

   }*/

		//robot.RechercheEtOrientation();
		//robot.moveForward();




	}

	public MoveScan(Deplacement mouvementRobot) {


		//Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, 0.056).offset(-0.06);  
		//the wheel diameter 56 mm, The offset specifies the position of
		//each wheel relative to the robot’s center.
		//Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 0.056).offset(0.06);  // Roue droite
		//Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel, rightWheel},WheeledChassis.TYPE_DIFFERENTIAL);

		deplacement = mouvementRobot;
		//deplacement = new Deplacement(chassis);
		ultrasonicSensor=new EV3UltrasonicSensor(SensorPort.S1);

	}

	public MoveScan(Deplacement mouvementRobot, Port s1) {

		deplacement = mouvementRobot;
		ultrasonicSensor=new EV3UltrasonicSensor(s1);

	}

	public double getDistance() {
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

	public int forward(double distance) {

		this.deplacement.avancerDe(distance);
		double d = getDistance();
		//deplacement.isMoving()

		while(deplacement.isMoving() && d>=30) {
			d=getDistance();
			System.out.println("la distance"+ d);
		}
		this.deplacement.stop();
		if (d <= 30) {
			return 0;
		} else {
			return 1; //
		}


	}

	public void  RechercheEtOrientation() {
		List<Double> distance = rotateMeasure();

		List<double[]> res=findAllDiscontinuities(distance, 40);
		double [] resFinal =findSmallestDiscontinuity(res);

		for (double[] arr : res) {
			System.out.println(Arrays.toString(arr));
		}
		System.out.println(Arrays.toString(resFinal));
		double angle=calc_angle(resFinal, distance.size());

		double targetDistance = resFinal[2];
		if (this.isPalet(resFinal, distance, 3.0, 25)) {

			double tolerance = 5; // 3 cm tolerance

			System.out.println("Bonne valeur (avant modif) : " +angle);
			if(targetDistance>=30 && targetDistance < 60) {
				angle+=10;

			} else if(targetDistance>=60 && targetDistance < 80) {
				angle+=10;
			}
			else {
				angle+=15;
			}

			if(angle>180) {
				//deplacement.rotate(360-angle, false);
				deplacement.pivoterDroite(360-angle);
				System.out.println("L'orientation finale : "+angle);
			}
			else {
				//deplacement.rotate(angle+15, false);
				deplacement.pivoterGauche(angle);
			}

			System.out.println("Bonne valeur (après modif) : " +angle);



			// Get the initial distance from the ultrasonic sensor
			//double currentDistance = getDistance();
			//System.out.println("Current distance: " + currentDistance);



			// First while loop

			boolean found=true;
			double a1 = this.getDistance();
			System.out.println("Distance Actuelle : " +a1);
			if (Double.isInfinite(a1) || (a1 - tolerance < a1
					&& a1 < a1 + tolerance)) {
				found=false;
			}
			System.out.println("Found : " +found);


			while(!found) {
				double currentDistance = getDistance();
				double rez=currentDistance - tolerance;
				//double rez2=currentDistance;
				System.out.println("differenca "+rez);
				System.out.println("Je suis dans le while");
				//System.out.println("Distance actuale: "+rez2);
				found= currentDistance - tolerance <= currentDistance
						&& currentDistance <= currentDistance + tolerance;
				if(angle>180) {
					deplacement.pivoterDroite(10);
				}
				else {
					deplacement.pivoterGauche(10);
				}

			}
			System.out.println(found);


		} else {
			//éviter
			deplacement.pivoterDroite(angle+180);
			deplacement.avancerDe(50);
			//relancer une recherche
		}
		// Stop the robot once aligned
		deplacement.stop();

	}

	public void moveForward() {
		double distance=getDistance();
		deplacement.avancerDe(distance);

		boolean distanceDecrease = true;
		while(deplacement.isMoving() && distanceDecrease) {
			double newDistance = getDistance();
			if (distance-newDistance<0) {
				distanceDecrease=false;

			}
		}
		deplacement.stop();

	}






	public List<Double> rotateMeasure() {
		List<Double> distance = new ArrayList<Double>();
		deplacement.setAngularSpeed(100);
		deplacement.rotate(370, true);
		while(deplacement.isMoving()) {
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
		deplacement.stop();
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
	public boolean isWall() {
		double distance=getDistance();
		if(distance>30) {
			return true;
		}
		return false;
	}
	public boolean isPalet (double[] smallestDiscontinuity, List<Double> distances,
			double stabilityThreshold, double lengthThreshold) {

		int firstIndex = (int) smallestDiscontinuity[0];
		int secondIndex = (int) smallestDiscontinuity[1];
		boolean stabilityCheck=isStableBetween(distances, firstIndex, secondIndex, stabilityThreshold);
		System.out.println("checking the stability "+ stabilityCheck);


		if (stabilityCheck) {

			double estimatedDistance = calculateObjectLength(distances, firstIndex, secondIndex);
			if (estimatedDistance <= lengthThreshold ) {
				return true;
			}
		}
		return false;

	}
	public String analyzeDiscontinuity(double[] smallestDiscontinuity, List<Double> distances, double stabilityThreshold, double lengthThreshold) {
		int firstIndex = (int) smallestDiscontinuity[0];
		int secondIndex = (int) smallestDiscontinuity[1];
		boolean stabilityCheck=isStableBetween(distances, firstIndex, secondIndex, stabilityThreshold);
		System.out.println("checking the stability "+ stabilityCheck);


		if (stabilityCheck) {

			double estimatedDistance = calculateObjectLength(distances, firstIndex, secondIndex);

			if (estimatedDistance > lengthThreshold )
			{
				return "Wall or robot detected";
			} else {
				return "Pallet detected";
			}
		}
		return "nothing found";

		/*

   // If not stable, check if the object is moving toward us
   else if (isObjectMovingToward(distances, firstIndex, secondIndex)) {
       return "Object moving toward detected";
   } else {
       return "Inconsistent readings; no wall or pallet detected";
   }
   <30 mur
		 */
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


	public void close() {
		ultrasonicSensor.close();
	}



}

