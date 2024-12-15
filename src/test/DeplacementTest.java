package test;

import action.Deplacement;
import lejos.hardware.port.MotorPort;

class DeplacementTest {

    
    void testAvancerDe() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        deplacement.avancerDe(50);
        System.out.println("Test de avancerDe exécuté.");
    }

    void testReculerDe() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        deplacement.reculerDe(20);
        System.out.println("Test de reculerDe exécuté.");
    }

   
    void testPivoterGauche() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        deplacement.pivoterGauche(90);
        System.out.println("Test de pivoterGauche exécuté.");
    }

    
    void testPivoterDroite() {
        Deplacement deplacement = new Deplacement(MotorPort.A, MotorPort.C);
        deplacement.pivoterDroite(90);
        System.out.println("Test de pivoterDroite exécuté.");
    }
}