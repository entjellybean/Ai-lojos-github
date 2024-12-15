package test;

import perception.ColorSensor;
import lejos.hardware.port.SensorPort;

class ColorSensorTest {

    void testDetecteLigneBlanche() {
        ColorSensor colorSensor = new ColorSensor(SensorPort.S4);
        boolean result = colorSensor.detecteLigneBlanche();
        System.out.println("Test de detecteLigneBlanche exécuté. Résultat : " + result);
    }

    void testClose() {
        ColorSensor colorSensor = new ColorSensor(SensorPort.S4);
        colorSensor.close();
        System.out.println("Test de close exécuté.");
    }

    public static void main(String[] args) {
        ColorSensorTest test = new ColorSensorTest();
        test.testDetecteLigneBlanche();
        test.testClose();
    }
}