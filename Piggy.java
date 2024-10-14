import lejos.hardware.Button;

public class Piggy {

	private int orientation;
	protected static Deplacement moteur;
	protected static Pince pince;



	public Piggy(int a) {
		orientation=a;
	}
	public Piggy() {
		this(90);
	}
	public static void main(String[] args) {
		// TODO Auto-generated method stub

		//Avancer forward
		moteur.avancer(40);//vérifier unité
		
		//Prendre le palet
		pince.recupererPalet();
		
		//Tourner droite (voir angle)
		moteur.tournerDroite(30);
		
		//avancer
		moteur.avancer(40);//vérifier unité
		
		//Tourner gauche (voir angle)
		moteur.tournerGauche(30);
		

		while (Button.ESCAPE.isUp()) {
			
			for(){
				//Avance tant que la ligne blanche n'est pas traversée
				//SI : Palet -> Eviter
				//SI : robot -> Eviter
				//SI : pas de ligne blanche -> Est-ce qu'on la traite ?
				
			}
			

			//Lacher palet
			pince.lacherApresLigneBlanche();
			
			//Recherche palet le plus proche

			//Orientation vers palet
			//+ MAJ attribut orientation

			//Avancer tant que on voit le palet
			//SI : Robot -> Eviter
			//Sinon : retourner à prendre palet
			
			//Prendre le palet
			pince.recupererPalet();
			

			//S'orienter vers la ligne blanche + retourne de combien on a bougé
			//orientation = moteur
			
			//+ MAJ attribut orientation
			
		}




	}
}

