package perception;

/**
 * La classe DistanceComparator permet de comparer deux tableaux de doubles
 * en se basant sur la valeur à l'indice 2. 
 * @author Narta Neziraj
 */
public class DistanceComparator {
    /**
     * Compare deux tableaux de doubles en fonction de la valeur située à l'indice 2.
     *
     * @param arr1 Premier tableau de doubles à comparer.
     * @param arr2 Deuxième tableau de doubles à comparer.
     * @return Un entier négatif si la valeur de arr1[2] est inférieure à arr2[2], 
     *         zéro si elles sont égales, ou un entier positif si arr1[2] est supérieure à arr2[2].
     * @throws ArrayIndexOutOfBoundsException si l'un des tableaux a une taille inférieure à 3.
     */

    public int compare(double[] arr1, double[] arr2) {

        return Double.compare(arr1[2], arr2[2]);

    }
}
