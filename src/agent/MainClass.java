package agent;
import lejos.hardware.Button;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

import java.util.Arrays;
import java.util.List;

import action.Deplacement;
import action.Pince;
import perception.BoutonConsole;
import perception.ColorSensor;
import perception.MoveScan;


/**
 * La classe MainClass implémente la stratégie du robot pour la compétition contre un robot adversaire.
 * 
 * <p>La stratégie suit plusieurs étapes :</p>
 * <ul>
 *   <li>Récupération des deux premiers palets selon des trajectoires prédéfinies.</li>
 *   <li>Recherche et manipulation des palets restants.</li>
 *   <li>Gestion dynamique des obstacles, comme les murs et le robot adverse.</li>
 *   <li>Retour à une position centrale après chaque dépôt pour optimiser les mouvements.</li>
 * </ul>
 * @author Développement : Basak Unal, Zoe Laget Thomas
 * @author Stratégie : Basak Unal, Narta Neziraj, Yassmina Cherqaoui, Zoe Laget Thomas
 */
public class MainClass {
	private Deplacement mouvementRobot; // Gère les déplacements du robot
	private Pince pince;  // Manipule les palets
	private ColorSensor colorSensor; // Détecte les lignes blanches
	private BoutonConsole boutonConsole;  // Gère les interactions utilisateur via les boutons
	private MoveScan moveScan; // Permet de scanner l'environnement pour détecter les palets
	private boolean isFirstPalletPicked = false; // Indique si le premier palet a été récupéré
	private boolean isSecondPalletPicked = false; // Indique si le deuxième palet a été récupéré
	private int positionRobotEnFace; / Position initiale du robot adverse (donnée par nous via le bouton)

	/**
        * Constructeur de la classe MainClass.
        * Configure tout le nécessaire pour exécuter la stratégie.
        */
	public MainClass() {
		Port A = LocalEV3.get().getPort("A");
		Port B = LocalEV3.get().getPort("B");
		Port C = LocalEV3.get().getPort("C");
		Port D = LocalEV3.get().getPort("D");
		Port s1 = LocalEV3.get().getPort("S1");
		//Port s2 = LocalEV3.get().getPort("S2");
		Port s3 = LocalEV3.get().getPort("S3");
		Port s4 = LocalEV3.get().getPort("S4");


		mouvementRobot = new Deplacement(A,C);
		colorSensor = new ColorSensor(s4);
		boutonConsole = new BoutonConsole();
		moveScan = new MoveScan(mouvementRobot,s1); 
		pince = new Pince(D,s3);

	}

	/**
        * Point d'entrée du programme.
        * Lance la stratégie de compétition pour récupérer et déposer les palets 
        * en suivant les étapes définies dans la méthode {@link #start()}.
        */
	public static void main(String[] args) {
		MainClass robot = new MainClass();
		robot.start();
	}

	
        /**
        * Exécute la stratégie principale du robot.
        * <p>Les étapes incluent :</p>
        * <ul>
        *   <li>Récupération et dépôt des deux premiers palets à des positions fixes.</li>
        *   <li>Recherche dynamique des palets restants.</li>
        *   <li>Évitement des obstacles (mur ou robot adverse).</li>
        * </ul>
        */
	public void start() {
		pince.fermer(1000); // Ferme la pince pour initialiser la position
		System.out.println("wait...");
		 // L'utilisateur indique la position du robot adverse via le bouton
		positionRobotEnFace = boutonConsole.positionRobot();

		// Boucle principale de la stratégie
		while (Button.ESCAPE.isUp()) {

			if (!isFirstPalletPicked) {
				// Récupération et dépôt du premier palet
				mouvementRobot.avancerDe(45);
				pince.lacherPalet();
				mouvementRobot.avancerDe(10);
				pince.recupererPalet();
				mouvementRobot.pivoterDroite(60);
				mouvementRobot.avancerDe(35);
				mouvementRobot.pivoterGauche(60);
				goToWhiteLineAndDrop1erPalet();
				isFirstPalletPicked = true;

			} else if(!isSecondPalletPicked) {
				// Récupération et dépôt du deuxième palet
				goToSecondPalletAndPick();
				isSecondPalletPicked = true;
				mouvementRobot.pivoterGauche(163);
				mouvementRobot.avancerDe(100);
			} else{
				 // Recherche et collecte des palets restants
				searchAndCollectNextPallet();
			}
		}


		System.out.println("stop.");
		cleanup();
	}

	/**
        * Déplace le robot vers la ligne blanche et dépose le premier palet.
        * Cette méthode utilise une trajectoire prédéfinie sans gestion d'obstacles.
        */
	private void goToWhiteLineAndDrop1erPalet() {
		mouvementRobot.avancerContinu(true); //avance jusqu'à la ligne blanche
		boolean res = false;
		System.out.println("Is moving : "+mouvementRobot.isMoving());
		while (mouvementRobot.isMoving() && !res) {
			res= colorSensor.detecteLigneBlanche();
			System.out.println(res);
		}
		mouvementRobot.stop();
		pince.lacherPalet();    

		// Recule après avoir déposé le palet
		mouvementRobot.reculerDe(10); // Recule de 20 cm (ajustez la distance si nécessaire)
		System.out.println("Recul après dépôt.");

		// Ferme la pince
		pince.recupererPalet();	    
		System.out.println("Pince fermée.");

	}

	/**
        * Une fois le palet récupéré, cette méthode permet d'aller le déposer à la ligne blanche.
        * 
        * <p>Elle contient une méthode pour éviter l'autre robot, car on utilise cette méthode 
        * pour la partie non déterministe.</p>
        */
	private void goToWhiteLineAndDrop() { 
		

		mouvementRobot.avancerContinu(true);
		boolean res = false;
		System.out.println("Is moving : "+mouvementRobot.isMoving());
		while (mouvementRobot.isMoving() && !res) { //regarder quand ça s'arrête
			res= colorSensor.detecteLigneBlanche();
			System.out.println(res);
			if (detectObstacle()) { //la différence c'est qu'on traite s'il y a un obstacle
				eviterObstacle();
			}
		}
		mouvementRobot.stop();
		pince.lacherPalet();   //deposer le palet 
		mouvementRobot.reculerDe(10); // Recule de 10 cm (ajuster la distance si nécessaire)
		System.out.println("Recul après dépôt.");
		pince.fermer(1000); // Ferme la pince après le recul
		System.out.println("Pince fermée.");

	}
	
	/**
        * La méthode a pour objectif de récupérer le 2ème palet et de le récupérer.
        * 
        * <p>Comme notre robot avait tendance à se déplacer sur la gauche, 
        * les angles ne sont pas symétriques pour compenser.</p>
        * 
        * <p>positionRobotEnFace est récupéré lors du démarrage du robot en appuyant 
        * sur le bouton correspondant par l'utilisateur en fonction de la position 
        * du robot en face.</p>
        * 
        * <p>s'oriente, va vers le robot et récupère le palet.</p>
        */
	private void goToSecondPalletAndPick() {
		
		if (positionRobotEnFace == 0) { // Robot adverse à gauche
			mouvementRobot.pivoterGauche(150);
			mouvementRobot.avancerDe(25);
			pince.lacherPalet();
			mouvementRobot.avancerDe(23);
			pince.recupererPalet();

		} else if (positionRobotEnFace == 1) { // Robot adverse au centre
			//si le robot est au milieu
			mouvementRobot.pivoterDroite(130);
			mouvementRobot.avancerDe(35);
			pince.lacherPalet();
			mouvementRobot.avancerDe(17);
			pince.recupererPalet();

			mouvementRobot.setOrientation(130);
		} else if (positionRobotEnFace == 2) { // Robot adverse a droite
			mouvementRobot.pivoterGauche(150);
			mouvementRobot.avancerDe(25);
			pince.lacherPalet();
			mouvementRobot.avancerDe(23);
			pince.recupererPalet();
		}
		
		if (positionRobotEnFace == 0) { //si le robot est à gauche
			mouvementRobot.orienterVersLigneAdverse();
			mouvementRobot.setOrientation(0);

		} else if (positionRobotEnFace == 1) { //si le robot est au milieu
			mouvementRobot.orienterVersLigneAdverse();
			mouvementRobot.setOrientation(0);
		} else if (positionRobotEnFace == 2) { //si le robot est à droite
			mouvementRobot.orienterVersLigneAdverse();
			mouvementRobot.setOrientation(0);
		}
		 // Orientation vers la ligne blanche
		goToWhiteLineAndDrop1erPalet();
		mouvementRobot.stop();
	}

	/**
	* Départ de la méthode au milieu de la table, effectue une recherche
	* Si la recherche est concluante (la 1ere ou 2ème discontinuité est un palet,
	* on va le récupérer, le déposé derrière la ligne blanche et retourné au milieu)
	* Sinon, au sein de la classe RechercheEtOrientation, on va lui demander de tourner
	* de 90° et d'avancer de 50 cm, le while du start permettre de relancer une nouvelle
	* recherche
	*/
	private void searchAndCollectNextPallet() {
		
		if(moveScan.RechercheEtOrientation()) {
			System.out.println("Il y a un palet");
			pince.lacherPalet();
			mouvementRobot.avancerDe(10);
			pince.recupererPalet();
			mouvementRobot.orienterVersLigneAdverse();
			goToWhiteLineAndDrop();
			mouvementRobot.reculerDe(10);
			mouvementRobot.pivoterGauche(163);
			mouvementRobot.avancerDe(100);
		}
	}

	/**
	* Si le capteur détect quelque chose en dessous de 25 cm, ça ne peut pas être un palet
	* Il faut donc l'éviter
 	*  @return true si un obstacle est détecté à moins de 25 cm, false sinon.
	*/
	private boolean detectObstacle() {
		if (moveScan.getDistance()>=25){
			return false;
		}
		return true;
	}

	/**
        * Évite un obstacle détecté en ajustant la trajectoire du robot.
        * <p>Gère deux cas :</p>
        * <ul>
        *   <li>Obstacle de type mur : le robot tourne à 90 degrés pour contourner.</li>
        *   <li>Obstacle de type robot : le robot ajuste sa position pour éviter la collision.</li>
        * </ul>
        */
	private void eviterObstacle() {
		
		double distanceCentre = moveScan.getDistance();
		mouvementRobot.pivoterGauche(30);
		double distanceGauche = moveScan.getDistance();
		mouvementRobot.pivoterDroite(30);
		double distanceDroite = moveScan.getDistance();

		double tolerance = 10;

		if(distanceCentre - tolerance <= distanceGauche
				&& distanceGauche <= distanceCentre + tolerance &&
				distanceCentre - tolerance <= distanceDroite
				&& distanceDroite <= distanceCentre + tolerance) {
			//Si c'est vrai, c'est un mur, on va l'éviter à 90 degrés
			mouvementRobot.pivoterDroite(90);
		} else {
			 // C'est un robot : ajuster la trajectoire
			boolean b = true;
			int cpt=30;
			mouvementRobot.pivoterDroite(30);
			while(b){
				mouvementRobot.pivoterDroite(10);
				cpt+=10;
				if (detectObstacle()){
					b = false;
				}
			}
			int avancer = (int) Math.sqrt(Math.pow(distanceCentre,2)+225) + 20;
			//calcul l'hypothénuse pour avoir la distance à parcourir pour éviter l'obstacle
			mouvementRobot.avancerDe(avancer);
			mouvementRobot.pivoterGauche(cpt);
		}

	}

	 /**
         * Arrête tous les composants du robot.
         */
	private void cleanup() {
		mouvementRobot.stop();
		moveScan.close();
		colorSensor.close();
		pince.close();
	}
}
