#ifndef SIMULATION_H
#define SIMULATION_H

//lit un fichier
void simulation_lecture (char nom_fichier[]);

//appelle les fonctions de dessin
void simulation_dessin();

//sauvegarde les donnees de la simulation courante dans un fichier
void save(char nom_fichier_sauv[]);

//deplace 1 robot
float simulation_deplacement(float translation_value, float rotation_value);

//selectionne un robot si il y en a un aux coordoonées indiquées
void simulation_select(float x,float y);

//deselectionne tous les robots
void deselection_robots();

#endif
