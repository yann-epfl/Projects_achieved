/**
 * \file		robot.c
 * \version		5.4
 * \date		2017-05-20
 * \author		Sélim Kamal 288567
 * \brief		module qui gère les robots (lecture, dessin, déplacement)
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "constantes.h"
#include "utilitaire.h"
#include "error.h"

#include "robot.h"

#define CHAMP_POS_X_ROB     1
#define CHAMP_POS_Y_ROB     2
#define CHAMP_POS_ALPHA     3

//etat
#define LIBRE				0
#define OCCUPE				1

//but
#define NOT_DEF			   -1

#define DIST_MAX			100

typedef struct robot ROBOT;
struct robot
{
	C2D cercle;
	double alpha;
	int objectif;
	int etat;
	int but;
	bool selected;
};

static char ligne_temp[LG_MAX_COORD];	
static bool fin_ligne;
static int i, type;
static int *p_i;
static double a, b;

static int nb_robots_ref;
static int nb_robots;
static ROBOT *tab_rob;

//remplit les champs de structure ROBOT avec les donnees du fichier
static void remplit_champ_rob(int champ, char ligne_temp[], int nb_robots,
							  ROBOT *tab_rob);

/*travaille avec une ligne de donnee ROBOT,
les lit et determine si le parametre alpha est correct*/
static bool invalid_rob_angle(char ligne[]);

//calcule la vitesse de rotation adaptée
double calcul_vrot(double angle);

//calcule la vitesse de translation adaptée
double calcul_vtran(double distance, double ecart_angle);

//trouve le robot le plus proche d'une particule pour l'attribution de but
int get_rob_proche(int num_part, C2D *tab_part_c2d);

//gère les collisions
void collision_r(int i, int nb_part, C2D *tab_part_c2d, double deplacement,
				 S2D pos_init);
				 
//trouve la particule la plus proche d'un robot pour l'attribution de but
int part_proche(C2D *tab_part_c2d, int num_rob, int nb_part);


void robots_deselection_robots()
{
	for(int i=0; i<nb_robots; i++)
	{
		tab_rob[i].selected = false;
	}
}

void deplacement_robot(int nb_part, C2D *tab_part_c2d, int i, float translation_value,
					   float rotation_value)
{	
	double v_tran = 0;
	double v_rot = 0;
	int j=tab_rob[i].but;
	if(util_distance(tab_rob[i].cercle.centre, tab_part_c2d[j].centre) >=
	   R_ROBOT+tab_part_c2d[j].rayon+EPSIL_ZERO)
		tab_rob[i].objectif = LOIN;
	if(tab_rob[i].etat == OCCUPE || tab_rob[i].selected == true)
	{	
		double ecart_angle;
		double *p_ecart_angle = &ecart_angle;
		double variation_angle;
		util_ecart_angle(tab_rob[i].cercle.centre, tab_rob[i].alpha,
						 tab_part_c2d[j].centre, p_ecart_angle);
		if(fabs(ecart_angle)>=EPSIL_ZERO || tab_rob[i].selected == true)
		{
			util_range_angle(p_ecart_angle);
			if(tab_rob[i].selected == true)
				v_rot = rotation_value;
			else
				v_rot = calcul_vrot(ecart_angle);
			variation_angle = v_rot * DELTA_T;
			tab_rob[i].alpha += variation_angle;
			util_range_angle(&(tab_rob[i].alpha));
		}
		if(tab_rob[i].objectif == LOIN)
		{
			double distance;
			double *p_distance = &distance;
			double deplacement;
			S2D pos_init;
			pos_init.x = tab_rob[i].cercle.centre.x;
			pos_init.y = tab_rob[i].cercle.centre.y;
			distance = util_distance(tab_rob[i].cercle.centre, tab_part_c2d[j].centre)
					   - tab_part_c2d[j].rayon - R_ROBOT; 
			if(tab_rob[i].selected == true)
				v_tran = translation_value;
			else
				v_tran = calcul_vtran(distance, ecart_angle);
			deplacement = v_tran * DELTA_T;
			double dep_x = deplacement * cos(tab_rob[i].alpha);
			double dep_y = deplacement * sin(tab_rob[i].alpha);
			tab_rob[i].cercle.centre.x += dep_x;
			tab_rob[i].cercle.centre.y += dep_y;
			if(util_distance(tab_rob[i].cercle.centre, tab_part_c2d[j].centre) <
			   R_ROBOT+tab_part_c2d[j].rayon+EPSIL_ZERO)
				tab_rob[i].objectif = COLLISION;
			collision_r(i, nb_part, tab_part_c2d, deplacement, pos_init);
		}
		j=tab_rob[i].but;
		util_ecart_angle(tab_rob[i].cercle.centre, tab_rob[i].alpha,
						 tab_part_c2d[j].centre, p_ecart_angle);
		if(tab_rob[i].objectif == COLLISION && fabs(ecart_angle)<EPSIL_ZERO)
			tab_rob[i].objectif = ALIGNE;
	}
}

int get_rob_proche(int num_part, C2D *tab_part_c2d)
{
	double distance_min = DIST_MAX;
	double ecart_angle_min = M_PI_2;
	double time_min = (ecart_angle_min/VROT_MAX) + (distance_min/VTRAN_MAX);
	
	double distance;
	double ecart_angle;
	double *p_ecart_angle = &ecart_angle;
	double time;
	
	int num_rob_proche = 0;
	int num_rob;
	bool particule_cible = false;
	
	for(int i=0; i<nb_robots; i++)
	{
		if(tab_rob[i].but == num_part)
		{
			particule_cible = true;
			num_rob_proche = i;
			break;
		}
	}
	
	if(!particule_cible)
	{
		for(int i=0; i<nb_robots; i++)
		{
			num_rob=i;
			
			distance = util_distance(tab_rob[num_rob].cercle.centre,
									 tab_part_c2d[num_part].centre);
			util_ecart_angle(tab_rob[num_rob].cercle.centre, tab_rob[num_rob].alpha,
							 tab_part_c2d[num_part].centre, p_ecart_angle);
			ecart_angle = fabs(ecart_angle);
			time = ecart_angle/VROT_MAX + distance/VTRAN_MAX;
			
			if(time <= time_min)
			{
				if(tab_rob[num_rob].etat == LIBRE)
				{
					time_min = time;
					num_rob_proche = num_rob;
				}
			}
		} 
	}
	return num_rob_proche;
}

void maj_buts(int nb_part, C2D *tab_part_c2d)
{
	int num_rob;
	int nb_rob_libre = nb_robots;
	for(int i=0; i<nb_robots; i++)
	{
		if(tab_rob[i].objectif != COLLISION)
		{
			tab_rob[i].objectif = LOIN;
			tab_rob[i].etat = LIBRE;
			tab_rob[i].but = NOT_DEF;
		}
		else
		{
			tab_rob[i].etat = OCCUPE;
			tab_rob[i].but = part_proche(tab_part_c2d, i, nb_part);
			nb_rob_libre--;
		}
	}
	for(int j=0; j<nb_part && j<nb_rob_libre; j++)
	{
		num_rob = get_rob_proche(j, tab_part_c2d);
		tab_rob[num_rob].etat = OCCUPE;
		tab_rob[num_rob].but = j;
	}
}

int part_proche(C2D *tab_part_c2d, int num_rob, int nb_part)
{
	int num_part_proche=0;
	double distance_min = util_distance(tab_rob[num_rob].cercle.centre,
										tab_part_c2d[num_part_proche].centre)
						 -tab_part_c2d[num_part_proche].rayon;

	double distance;
	int num_part;
	
	for(int i=1; i<nb_part; i++)
	{
		num_part = i;
		distance = util_distance(tab_rob[num_rob].cercle.centre,
								 tab_part_c2d[num_part].centre);
		distance -= tab_part_c2d[num_part].rayon;
		
		if(distance < distance_min)
		{
			distance_min = distance;
			num_part_proche = num_part;
		}
	} 
	return num_part_proche;
}

void collision_r(int i, int nb_part, C2D *tab_part_c2d, double deplacement,
				 S2D pos_init)
{
	double dist_init;
	double distance;
	double * p_dist = &distance;
	double somme_r;
	double depl_regl;
	double *p_depl_regl = &depl_regl;
	
	for(int j=0; j<nb_robots; j++)
	{
		if(j != i)
		{
			somme_r = 2*R_ROBOT;
			if(util_collision_cercle(tab_rob[i].cercle, tab_rob[j].cercle, p_dist))
			{
				dist_init = util_distance(pos_init, tab_rob[j].cercle.centre);
				util_inner_triangle(deplacement, distance, dist_init, somme_r,
									p_depl_regl);
				double dep_x = (deplacement-depl_regl) * cos(tab_rob[i].alpha);
				double dep_y = (deplacement-depl_regl) * sin(tab_rob[i].alpha);
				tab_rob[i].cercle.centre.x -= dep_x;
				tab_rob[i].cercle.centre.y -= dep_y;
			}
		}
	}
	
	for(int j=0; j<nb_part; j++)
	{
		somme_r = R_ROBOT + tab_part_c2d[j].rayon;
		if(util_collision_cercle(tab_rob[i].cercle, tab_part_c2d[j], p_dist))
		{
			dist_init = util_distance(pos_init, tab_part_c2d[j].centre);
			util_inner_triangle(deplacement, distance, dist_init, somme_r,
								p_depl_regl);
			double dep_x = (deplacement-depl_regl) * cos(tab_rob[i].alpha);
			double dep_y = (deplacement-depl_regl) * sin(tab_rob[i].alpha);
			tab_rob[i].cercle.centre.x -= dep_x;
			tab_rob[i].cercle.centre.y -= dep_y;
			tab_rob[i].but=j;
			tab_rob[i].objectif=COLLISION;
		}
	}
}

double calcul_vtran(double distance, double ecart_angle)
{
	if(fabs(ecart_angle)<M_PI/2)
	{
		if(distance/DELTA_T > VTRAN_MAX)
		{
			return VTRAN_MAX;
		}
		else
		{
			return distance/DELTA_T;
		}
	}
	return 0;
}

double calcul_vrot(double angle)
{
	if(angle/DELTA_T > VROT_MAX)
		return VROT_MAX;

	if(angle/DELTA_T < -VROT_MAX)
		return -VROT_MAX;
		
	return angle/DELTA_T;
}

bool lecture_robot(char ligne[], unsigned int num_ligne, int *p_struct_type)
{
	fin_ligne = false;
	i=0;
	p_i = &i;
	type = sscanf(ligne, "%lf %lf", &a, &b);
	switch(type)
	{
		case FIN_LISTE:
			*p_struct_type = PARTICULE;
			if(nb_robots<nb_robots_ref)
			{
				error_fin_liste_robots(num_ligne);
				reinit_rob();
				return true;
			}
			break;
		case NB_REF:
			nb_robots_ref = a;
			break;
		case DONNEES:
			if(tab_rob == NULL)
				tab_rob = malloc(nb_robots_ref*sizeof(ROBOT));
			
			if(invalid_rob_angle(ligne))
			{
				error_invalid_robot_angle(tab_rob[nb_robots-1].alpha);
				reinit_rob();
				return true;
			}
			
			if(!fin_ligne)
			{
				if(reste_donn(ligne, i))
				{
					error_missing_fin_liste_robots(num_ligne);
					reinit_rob();
					return true;
				}
			}
			break;
	}
	return false;
}

void remplit_champ_rob(int champ, char ligne_temp[], int nb_robots, ROBOT *tab_rob)
{
	switch(champ)
	{
		case CHAMP_POS_X_ROB:
			sscanf(ligne_temp, "%lf", &tab_rob[nb_robots].cercle.centre.x);
			break;
		case CHAMP_POS_Y_ROB:
			sscanf(ligne_temp, "%lf", &tab_rob[nb_robots].cercle.centre.y);
			break;
		case CHAMP_POS_ALPHA:
			sscanf(ligne_temp, "%lf", &tab_rob[nb_robots].alpha);
			break;
	}
	tab_rob[nb_robots].cercle.rayon = R_ROBOT;
	tab_rob[nb_robots].objectif = LOIN;
	tab_rob[nb_robots].etat = LIBRE;
	tab_rob[nb_robots].but = NOT_DEF;
	tab_rob[nb_robots].selected = false;
}

bool invalid_rob_angle(char ligne[])
{
	while(!fin_ligne && nb_robots<nb_robots_ref)
	{
		for(int champ=1; champ<=CHAMP_POS_ALPHA; champ++)
		{
			if(donnee(ligne, p_i))
			{
				remplit_ligne_temp(ligne, p_i, ligne_temp);
				remplit_champ_rob(champ, ligne_temp, nb_robots, tab_rob);
				if(champ == CHAMP_POS_ALPHA)
				{
					nb_robots++;
				}
			}
			else
			{
				fin_ligne = true;
				break;
			}
		}
		if(util_alpha_dehors(tab_rob[nb_robots-1].alpha))
		{
			return true;
		}
	}
	return false;
}

int get_nb_rob()
{
	return nb_robots;
}

C2D* get_tab_rob_c2d()
{
	C2D *tab_rob_c2d;
	tab_rob_c2d = malloc(nb_robots_ref*sizeof(C2D));
	for(int i=0; i<nb_robots_ref; i++)
	{
		tab_rob_c2d[i].centre.x=tab_rob[i].cercle.centre.x;
		tab_rob_c2d[i].centre.y=tab_rob[i].cercle.centre.y;
		tab_rob_c2d[i].rayon=tab_rob[i].cercle.rayon;
	}
	return tab_rob_c2d;
}

int get_obj(int i)
{
	return tab_rob[i].objectif;
}

int ret_but(int i)
{
	return tab_rob[i].but;
}

void draw_robots()
{
	for(int i=1; i<=nb_robots; i++)
	{
		if(!tab_rob[i-1].selected)
		{
			dessin_robot(tab_rob[i-1].cercle.centre.x, tab_rob[i-1].cercle.centre.y,
						 tab_rob[i-1].alpha);
		}
		else
		{
			dessin_robot_selection(tab_rob[i-1].cercle.centre.x, 
			                       tab_rob[i-1].cercle.centre.y,tab_rob[i-1].alpha);
		}
	}
}

void reinit_rob()
{
	free(tab_rob);
	tab_rob = NULL;
	nb_robots_ref = 1;
	nb_robots = 0;
}

void save_rob(FILE* p_fichier_sauv)
{
	fprintf(p_fichier_sauv, "%d\n", nb_robots);
	for(int i=0; i<nb_robots; i++)
	{
		fprintf(p_fichier_sauv, "	%lf %lf %lf\n", tab_rob[i].cercle.centre.x,
				tab_rob[i].cercle.centre.y, tab_rob[i].alpha);
	}
	fprintf(p_fichier_sauv, "FIN_LISTE\n");
}

void selection_robot(float x, float y)
{
	float distance;
	S2D selection;
	selection.x = x;
	selection.y = y;
	
	for(int i=0; i<nb_robots; i++)
	{
		distance = util_distance(selection, tab_rob[i].cercle.centre);
		if(distance <= R_ROBOT)
		{
			if(tab_rob[i].selected == false)
				tab_rob[i].selected = true;
			else
				tab_rob[i].selected = false;
		}
		else
			tab_rob[i].selected = false;
	}
}
