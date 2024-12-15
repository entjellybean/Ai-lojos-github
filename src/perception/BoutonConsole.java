package perception;
import lejos.hardware.Button;

import lejos.utility.Delay;

/**
 * La classe BoutonConsole permet d'interagir avec les boutons de la brique EV3 
 * pour déterminer la position du robot sur le terrain.
 * 
 * <p>Cette classe affiche un message sur la console et attend que l'utilisateur 
 * appuie sur un bouton pour sélectionner une position.</p>
 * @author Zoe Laget Thomas
 */
public class BoutonConsole {

	
	/**
       * Demande à l'utilisateur d'indiquer la position de l'adversaire via les boutons de la brique EV3.
       * 
       * <p>Correspondances des boutons :</p>
       * <ul>
       *   <li>"ENTER" : Centre (1).</li>
       *   <li>"LEFT" : Gauche (0).</li>
       *   <li>"RIGHT" : Droite (2).</li>
       *   <li>Retourne 4 par défaut si aucun bouton valide n'est pressé.</li>
       * </ul>
       *
       * @return Un entier indiquant la position (0 : Gauche, 1 : Centre, 2 : Droite, 4 : Par défaut).
       */
	public int positionRobot() {

		//renvoie un entier en fonction de la position
		
		System.out.println("Ou est l'adversaire ?");
		Button.waitForAnyPress(); // Attend la pression d'un bouton

		boolean b = true;
		while (b) {
			if (Button.ENTER.isDown()) {
				//Le robot au centre
				b =false;
				return 1;
			} else if (Button.LEFT.isDown()) {
				//Le robot à gauche
				b =false;
				return 0;
			} else if (Button.RIGHT.isDown()) {
				//Le robot à droite
				b =false;
				return 2;
			}

		}
		return 4;
	}
}
