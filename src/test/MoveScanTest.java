package test;

import action.Deplacement;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import perception.MoveScan;

public class MoveScanTest {

    public void testGetDistance() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        MoveScan moveScan = new MoveScan(deplacement, SensorPort.S1);

        double distance = moveScan.getDistance();
        System.out.println("Test getDistance executed. Distance measured: " + distance + " cm");
    }

    public void testRotateMeasure() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        MoveScan moveScan = new MoveScan(deplacement, SensorPort.S1);

        System.out.println("Test rotateMeasure executed. Robot rotating and measuring...");
        moveScan.rotateMeasure();
        System.out.println("Rotation complete. Distance measurements collected.");
    }

    public void testRechercheEtOrientation() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        MoveScan moveScan = new MoveScan(deplacement, SensorPort.S1);

        System.out.println("Test RechercheEtOrientation executed.");
        boolean result = moveScan.RechercheEtOrientation();
        System.out.println("Result of RechercheEtOrientation: " + result);
    }

    public void testFindAllDiscontinuities() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        MoveScan moveScan = new MoveScan(deplacement);

        System.out.println("Test findAllDiscontinuities executed.");
        moveScan.findAllDiscontinuities(moveScan.rotateMeasure(), 5.0);
        System.out.println("Discontinuities detected and processed.");
    }

    public static void main(String[] args) {
        MoveScanTest test = new MoveScanTest();

        System.out.println("Running tests for MoveScan...");
        test.testGetDistance();
        test.testRotateMeasure();
        test.testRechercheEtOrientation();
        test.testFindAllDiscontinuities();
        System.out.println("All tests for MoveScan completed.");
    }
}