#ifndef PARTICULE_H
#define PARTICULE_H

//lit les donnees des particules du fichier
bool lecture_particule(char ligne[], unsigned int num_ligne);

//transmet le nombre de particules lues
int get_nb_part();

//transmet le tableau contenant les donnees C2D des particules lues
C2D* get_tab_part_c2d();

//dessine une particule
void draw_particules();

//reinitialise toutes les donnees de particules liees à la lecture du fichier
void reinit_part();

//sauvegarde les donnees particule de la simulation courante dans un fichier
void save_part(FILE* p_fichier_sauv);

//detruie une particule donnée
void destroy_part(int j);

//Renvoie le taux de decontamination
float get_taux_dec();

//Decompose une particule
void split_part(int j);

//Renvoie le nombre d'identifiants de particule distincts
int get_nb_id();

#endif
