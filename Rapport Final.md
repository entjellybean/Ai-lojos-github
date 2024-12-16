## Rapport final 

1. Bilan du projet : 

Le projet s’est déroulé sans accros. Il nous a permis de développer des capacités de résolution de problèmes et d’adaptation aux défaillances ou limitations du robot. Par exemple, pour récupérer le second palet, dans notre stratégie nous déposions le premier palet entre la ligne vertical centrale et celle de droite au juste milieu, puis après l’avoir déposé nous n’avions plus qu’à nous orienter vers le second palet à droite ou à gauche avec le même angle. Sauf que notre robot avait tendance à avancer en prenant de l’angle vers la gauche au fur et à mesure. Aussi nous nous retrouvions avec un robot qui arrivait pratiquement au croisement entre la ligne verticale centrale et la ligne d’arrivée blanche, donc il a fallu adapter l’angle et la distance entre le robot et le palet, qui n’était alors plus symétrique.  
Une autre situation où l’on a dû faire preuve d’adaptation a été avec la classe moveScan, qui a pour objectif de localiser le prochain palet et de s’orienter vers celui-ci. En effet, après de nombreux tests nous avons du faire un certain nombre d’adaptation de l’angle trouvé, car notre robot tournait plus de 360 lorsque l’on lui disait de tourner de 360, et ne s'orientait pas exactement du bon angle. Donc pour combler ces problèmes, nous avons mis en place une boucle au sein de RechercheEtOrientation() pour adapter le tir et réussir à bien s’orienter.  
L’un des difficultés en plus de situation où l’adaptation était nécessaire, c’était la faible disponibilité de la table de passation, qui était constamment occupée, aussi de nombreux tests que nous effectuons à côté s'avérait avoir des résultats différents sur la table (comme la classe moveScan), aussi, il a fallu adapter certains codes et certes corrections dû aux imprécisions du robot lors des dernières semaines en urgence, ce qui n’était pas le plus simple.  
Aussi, pour conclure plus généralement sur la maîtrise de ce projet, nous avons commencé très rapidement à travailler sur la programmation des classes mais nos tests ont été plus tardif et pas toujours sur la table de passation ce qui a pu ajouter du bruit et donc mal adapter nos codes légèrement. Cependant, nous avons travaillées en continu lors de ce semestre et avons réussi à avoir un robot assez fonctionnel avec des classes qui fonctionnait. Le principal bémol est que l’on aurait du commencer à tester nos classes plus tôt pour avoir le temps de faire des ajustements pour être plus performant le jour du concours.  
Ainsi, malgré le fait que notre code n’était pas parfaitement abouti, nous avons su réaliser un robot qui était capable de récupérer des palets, aller jusqu’à la ligne blanche et faire une recherche relativement performante en prenant soin d’éviter les obstacles, ce qui est déjà une réussite en soit.  
Nous avons terminé la compétition à la 4ème place, avec 2 victoires sur 4 matchs. Voici notre analyse des points positifs et des points à améliorer :

### Points Positifs

* Recherche du palet : Notre méthode de recherche a bien fonctionné. Le robot a réussi à identifier le palet lorsque celui-ci était présent.  
* Évitement d'Obstacles : Le robot a correctement évité les murs à proximité tout en continuant sa recherche.  
* Détection de Couleur : Notre capteur de couleur a efficacement détecté la couleur blanche lors du dépôt d'une palette.

### Points à Améliorer

* Précision de l'Angle : Nous avons rencontré des problèmes de précision concernant l'angle calculé. Nous n'avons pas ajusté cet angle après le calcul, faute de temps pour tester des valeurs adéquates pendant la phase d'entraînement. Cet ajustement aurait pu nous permettre de remporter des victoires supplémentaires (au moins 6 points lors de l’un des matchs)..  
* Tests en Condition Réelle : Nous aurions souhaité effectuer davantage de tests sur la table de compétition. Les seuils de calcul et d'identification ont été établis sans entraînement adéquat, ce qui a limité notre capacité à affiner notre stratégie avant la compétition.

### Conclusion

Dans l'ensemble, notre code suivait une logique solide, et les tests réalisés avant le concours ont permis d’identifier avec succès le palet ou le mur. Nous pensons qu'avec des ajustements appropriés et plus de tests, nous aurions pu améliorer nos performances.

