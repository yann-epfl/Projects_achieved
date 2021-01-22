/**
 * \file		simulation.c
 * \version		4.0
 * \date		2017-05-20
 * \author		Boudigou Yann 284597
 * \brief		module de haut niveau qui gère la simulation dans son ensemble
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "constantes.h"
#include "utilitaire.h"
#include "error.h"
#include "robot.h"
#include "particule.h"

#include "simulation.h"

static FILE *p_fichier;
static FILE *p_fichier_sauv;
static char ligne[MAX_LINE];
static int *p_struct_type;
static int struct_type;
static unsigned int num_ligne;
static bool error;

//détermine si il y a une collision
static bool collision();

void simulation_lecture (char nom_fichier[])
{
	error = false;
	num_ligne = 0;
	p_fichier = fopen(nom_fichier, "r");
	struct_type = ROB;
	p_struct_type = &struct_type;
	
	reinit_rob();
	reinit_part();
	
	while(fgets(ligne, MAX_LINE, p_fichier) != NULL && !error)
	{
		num_ligne++;
		if(!commentaire(ligne))
		{
			if(*p_struct_type==ROB)
				error = lecture_robot(ligne, num_ligne, p_struct_type);
			else
				error = lecture_particule(ligne, num_ligne);
		}
	}
	
	if(!error)
	{
		if(collision())
		{
			reinit_rob();
			reinit_part();
			error = true;
		}
	}
	
	if(!error)
		error_no_error_in_this_file();
	
	fclose(p_fichier);
}

bool collision()
{
	int nb_rob = get_nb_rob();
	C2D *tab_rob_c2d = get_tab_rob_c2d();
	int nb_part = get_nb_part();
	C2D *tab_part_c2d = get_tab_part_c2d();
	double distance;
	double * p_dist = &distance;
	if(nb_rob>1)
	{
		int i,j;
		for(i=1; i<=nb_rob; i++)
		{
			for(j=i+1; j<=nb_rob; j++)
			{
				if(util_collision_cercle(tab_rob_c2d[i-1], tab_rob_c2d[j-1], p_dist))
				{
					error_collision(ROBOT_ROBOT, i, j);
					return true;
				}
			}
		}
	}
	if(nb_part>1)
	{
		int i,j;
		for(i=1; i<=nb_part; i++)
		{
			for(j=i+1; j<=nb_part; j++)
			{
				if(util_collision_cercle(tab_part_c2d[i-1], tab_part_c2d[j-1], p_dist))
				{
					error_collision(PARTICULE_PARTICULE, i, j);
					return true;
				}
			}
		}
	}
	if(nb_rob>0 && nb_part>0)
	{
		int i,j;
		for(i=1; i<=nb_rob; i++)
		{
			for(j=1; j<=nb_part; j++)
			{
				if(util_collision_cercle(tab_rob_c2d[i-1], tab_part_c2d[j-1], p_dist))
				{
					error_collision(ROBOT_PARTICULE, i, j);
					return true;
				}
			}
		}
	}
	free(tab_rob_c2d);
	free(tab_part_c2d);
	return false;
}

void simulation_dessin()
{
	if(!error)
	{
		cadre();
		draw_robots();
		draw_particules();
	}
}

void save(char nom_fichier_sauv[])
{
	p_fichier_sauv = fopen(nom_fichier_sauv, "w");
	
	save_rob(p_fichier_sauv);
	save_part(p_fichier_sauv);
	
	fclose(p_fichier_sauv);
}

void deselection_robots()
{
	robots_deselection_robots();
}

float simulation_deplacement(float translation_value, float rotation_value)
{
	int nb_rob = get_nb_rob();
	
	maj_buts(get_nb_part(), get_tab_part_c2d());
	
	for(int i=0; i<nb_rob; i++)
	{
		if(get_nb_part()) //si reste des particules
		{
			deplacement_robot(get_nb_part(), get_tab_part_c2d(), i, translation_value,
			                  rotation_value);
			
			if(get_obj(i) == ALIGNE && ret_but(i)!=-1)
			{
				destroy_part(ret_but(i));
				maj_buts(get_nb_part(), get_tab_part_c2d());
			}
		}
	}
	
	int nb_id = get_nb_id();
	
	if(get_nb_part())
	{
		for(int id=1; id<=nb_id; id++)
		{
			float decomposition = ((float)(rand()))/RAND_MAX;
			if(decomposition <= DECOMPOSITION_RATE)
				split_part(id);
		}
	}
	
	float taux_dec = get_taux_dec();
	return taux_dec;
}

void simulation_select(float x,float y)
{
	selection_robot(x, y);
}
