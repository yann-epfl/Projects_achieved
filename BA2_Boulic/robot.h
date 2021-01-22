#ifndef ROBOT_H
#define ROBOT_H

//lit les donnees des robots du fichier
bool lecture_robot(char ligne[], unsigned int num_ligne, int *p_struct_type);

//transmet le nombre de robots lus
int get_nb_rob();

//transmet le tableau contenant les donnees C2D des robots lues
C2D* get_tab_rob_c2d();

//transmet l'objectif du robot
int get_obj(int i);

//transmet le but du robot
int ret_but(int i);

//dessine un robot
void draw_robots();

//reinitialise toutes les donnees de robots liees à la lecture du fichier
void reinit_rob();

//sauvegarde les donnees robot de la simulation courante dans un fichier
void save_rob(FILE* p_fichier_sauv);


//se charge du deplacement d'un robot
void deplacement_robot(int nb_part, C2D *tab_part_c2d, int i, float translation_value,
					   float rotation_value);

//met à jour le but de tous les robots
void maj_buts(int nb_part, C2D *tab_part_c2d);

//determine quel robot est sélectionné en mode manuel (s'il y en a un)
void selection_robot(float x, float y);

//déselectionne tous les robots
void robots_deselection_robots();

#endif
