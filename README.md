# Introduction :

Bonjour et bienvenue dans le répertoire du projet « L’Homme et l’Oiseau » !

Vous trouverez dans ce fichier des instructions sur comment compiler le projet et des informations sur les fichiers contenus dans ce projet ainsi que leurs emplacements.

:warning: **Remarque :** :warning:

> Pour ceux qui sont familiers avec le fonctionnement de la bibliothèque VCL, certaines explications paraitront triviales puisque ce projet se base uniquement sur cette bibliothèque et adhère à son fonctionnement.

 ## Prérequis :

* Linux/Ubuntu/MaxOs
* git lfs

## Compilation et exécution du projet :

#### Uniquement sur Linux :

Pour exécuter le projet, il suffit d'ouvrir votre terminal dans le répertoire `Code_of_the_Human_and_the_Bird `  et faire un `cmake CMakeLists.txt ` suivi de `make `  et de ` ./pgm ` pour avoir la scène finale ouverte devant vous.  Cependant, je vous préviens qu’elle prend du temps à charger et qu’il faut dézoomer tant que vous ne voyez pas de texture apparaître (maintenez le bouton droit de la souris puis allez vers le haut.)

**Tutoriel détaillé en anglais** pour installer les prérequis et compiler sous :

* [Linux/Ubuntu](doc/compilation.md#Ubuntu)
* [MacOS](doc/compilation.md#MacOS)

Pour le reste des systèmes d'exploitations je ne sais pas faire XD.

#### Manipulations de la caméra :

* Le bouton gauche maintenu + déplacement de la souris permet de **changer l'orientation de la caméra**.
* Le bouton droit + déplacement de la souris vers le haut/bas permet de **zoomer/dézoomer**.
* CTRL + clic gauche + déplacement de souris permet de **translater le repère de la scène 3D.**

 


## Cartographie du projet :

À partir du répertoire principal ` The_Human_and_the_Bird `, vous pouvez accéder aux fichiers qui composent la dernière scène en allant dans le répertoire ` Code_of_the_Human_and_the_Bird/scenes/3D_graphics/05_final_scene `.

Voici une liste exhaustive des fichiers dans ` 05_final_scene ` :

1. **bench_hierarchy.cpp**, où la hiérarchie du banc est créée.
2. **bird_hierarchy.cpp**, où la hiérarchie de l’oiseau est créée. 
3. **final_scene.hpp/cpp**, où la scène est assemblée dans son intégralité. Mot clé de la scène : ` SCENE_FINAL_SCENE `.
4. **human_hierarchy.cpp**, où la hiérarchie de l’homme est créée.
5. **interpolation_functions.cpp**, où vous trouverez les fonctions utilisées pour créer les key frames des déplacements de l’homme et de l’oiseau, la fonction d’interpolation du mouvement et les fonctions qui calculent les rotations selon la direction de leurs déplacements.
6. **terrain_functions.cpp**, où vous trouverez les fonctions utilisées pour créer le terrain et les billboards.
7. **tree_modeling_functions.cpp**, où vous trouverez uniquement les fonctions utilisées pour créer la structure des arbres.

 


## Différentes scènes intéressantes :

Dans le répertoire `scenes` dans ` Code_of_the_Human_and_the_Bird `, vous trouverez un fichier qui s’appelle **current_scene.hpp** qui contient un champ **#define SCENE_FINAL_SCENE**. Vous pouvez remplacer ` SCENE_FINAL_SCENE` par n’importe quel autre champ de votre choix commenté en dessous.

Voici quelques champs intéressants :

1. `SCENE_ARTICULATED_HIERARCHY`, où figure seulement l’animation de l’homme.
2. `SCENE_BENCH_HIERARCHY`, où figure seulement le modèle 3D du banc (je vous le recommande, c’est mon préféré :P )
3. `SCENE_TREE_MODELING`, où figure seulement le modèle 3D de l’arbre.

 


## Médias :

Vous trouverez aussi dans les fichiers du projet des dossiers contenants des photos et vidéos des animations 3D faites dans ce projet, respectivement dans `Photos` et `Videos`, accessibles dans le dossier principal ` The_Human_and_the_Bird `.



## Auteur :

[Jean-Charles  LAYOUN](https://www.linkedin.com/in/jclayoun). Vous pouvez me contacter @  [jean-charles.layoun@polytechnique.edu](mailto:jean-charles.layoun@polytechnique.edu).

