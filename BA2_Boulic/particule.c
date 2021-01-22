/**
 * \file		particule.c
 * \version		6.2
 * \date		2017-05-20
 * \author		Boudigou Yann 284597
 * \brief		module qui gère les particules (lecture, dessin, décomposition)
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "constantes.h"
#include "utilitaire.h"
#include "error.h"

#include "particule.h"

#define CHAMP_ENERGY_PART   1
#define CHAMP_RAYON_PART    2
#define CHAMP_POS_X_PART    3
#define CHAMP_POS_Y_PART    4

#define HAUT_G   			1
#define HAUT_D   			2
#define BAS_G    			3
#define BAS_D    			4

#define POURCENTAGE			100

typedef struct part PART;
struct part
{
	int id;
	double energy;
	C2D cercle;
	PART* suivant;
};

static char ligne_temp[LG_MAX_COORD];	
static bool fin_ligne;
static int i, type;
static int *p_i;
static double a, b;

static int nb_parts_ref;
static int nb_parts;
static PART *p_list_part = NULL;
static PART *p_part;

static double somm_e_init;
static double somm_e_detr;

static int nb_id;

//remplit les champs de structure PART avec les donnees du fichier
static void remplit_champ_part(int champ, char ligne_temp[], PART *p_part);
/*travaille avec une ligne de donnee PART,
les lit et determine si les parametres sont corrects*/
bool invalid_part_value(char ligne[]);

static PART* search_part_num(int j);
static void insert_part(PART *new_p);
static void retirer_part(PART *p_part);
static void vider_liste();

bool lecture_particule(char ligne[], unsigned int num_ligne)
{
	fin_ligne = false;
	i=0;
	p_i = &i;
	type = sscanf(ligne, "%lf %lf", &a, &b);
	switch(type)
	{
		case FIN_LISTE:
			if(nb_parts<nb_parts_ref)
			{
				error_fin_liste_particules(num_ligne);
				reinit_part();
				return true;
			}
			break;
		case NB_REF:
			nb_parts_ref = a;
			break;
		case DONNEES:
			if(invalid_part_value(ligne))
			{
				error_invalid_particule_value(p_part->energy,
										  p_part->cercle.rayon,
										  p_part->cercle.centre.x,
										  p_part->cercle.centre.y);
				reinit_part();
				return true;
			}
			if(!fin_ligne)
			{
				if(reste_donn(ligne, i))
				{
					error_missing_fin_liste_particules(num_ligne);
					reinit_part();
				    return true;
				}
			}
			break;
	}
	return false;
}

void remplit_champ_part(int champ, char ligne_temp[], PART *p_part)
{
	switch(champ)
	{
		case CHAMP_ENERGY_PART:
			sscanf(ligne_temp, "%lf", &p_part->energy);
			break;
		case CHAMP_RAYON_PART:
			sscanf(ligne_temp, "%lf", &p_part->cercle.rayon);
			break;
		case CHAMP_POS_X_PART:
			sscanf(ligne_temp, "%lf", &p_part->cercle.centre.x);
			break;
		case CHAMP_POS_Y_PART:
			sscanf(ligne_temp, "%lf", &p_part->cercle.centre.y);
			break;
	}
}

bool invalid_part_value(char ligne[])
{
	while(!fin_ligne && nb_parts<nb_parts_ref)
	{
		p_part = malloc(sizeof(PART));
		for(int champ=1; champ<=CHAMP_POS_Y_PART; champ++)
		{
			if(donnee(ligne, p_i))
			{
				remplit_ligne_temp(ligne, p_i, ligne_temp);
				remplit_champ_part(champ, ligne_temp, p_part);
				if(champ == CHAMP_POS_Y_PART)
				{
					somm_e_init += p_part->energy;
					insert_part(p_part);
				}
			}
			else
			{
				fin_ligne = true;
				free(p_part);
				p_part = NULL;
				break;
			}
		}
		if(p_part != NULL)
		{
			if(util_point_dehors(p_part->cercle.centre, DMAX) ||
			   p_part->energy<0 ||
			   p_part->energy>E_PARTICULE_MAX ||
			   p_part->cercle.rayon<R_PARTICULE_MIN ||
			   p_part->cercle.rayon>R_PARTICULE_MAX)
			{
				return true;
			}
		}
	}
	return false;
}

float get_taux_dec()
{
	float taux_dec = (float)((somm_e_detr/somm_e_init)*POURCENTAGE);
	return taux_dec;
}

int get_nb_part()
{
	return nb_parts;
}

int get_nb_id()
{
	return nb_id;
}

C2D* get_tab_part_c2d()
{
	C2D *tab_part_c2d;
	p_part = p_list_part;
	tab_part_c2d = malloc(nb_parts*sizeof(C2D));
	for(int i=0; i<nb_parts; i++)
	{
		tab_part_c2d[i].centre.x=p_part->cercle.centre.x;
		tab_part_c2d[i].centre.y=p_part->cercle.centre.y;
		tab_part_c2d[i].rayon=p_part->cercle.rayon;
		p_part=p_part->suivant;
	}
	return tab_part_c2d;
}

void draw_particules()
{
	p_part = p_list_part;
	for(int i=0; i<nb_parts; i++)
	{
		dessin_part(p_part->cercle.centre.x, p_part->cercle.centre.y,
					p_part->cercle.rayon);
		p_part=p_part->suivant;
	}
}

void reinit_part()
{
	vider_liste();
	nb_parts_ref = 1;
	nb_parts = 0;
	somm_e_init = 0;
	somm_e_detr = 0;
	nb_id = 0;
}

void save_part(FILE* p_fichier_sauv)
{
	p_part = p_list_part;
	fprintf(p_fichier_sauv, "%d\n", nb_parts);
	for(int i=0; i<nb_parts; i++)
	{
		fprintf(p_fichier_sauv, "	%lf %lf %lf %lf\n", p_part->energy,
				p_part->cercle.rayon, p_part->cercle.centre.x,
				p_part->cercle.centre.y);
		p_part=p_part->suivant;
	}
	fprintf(p_fichier_sauv, "FIN_LISTE\n");
}

PART* search_part_num(int j)
{
	p_part = p_list_part;
	int i=0;
	while(i<j && i<nb_parts)
	{
		p_part=p_part->suivant;
		i++;
	}
	return p_part;
}

void destroy_part(int j)
{
	p_part=search_part_num(j);
	somm_e_detr += p_part->energy;
		
	retirer_part(p_part);
}

PART* search_part_id(int id)
{
	p_part = p_list_part;
	while(p_part->id != id && p_part->suivant)
		p_part=p_part->suivant;
	
	if(p_part->id == id)
		return p_part;
	else
	return NULL;

}

void split_part(int id)
{
	PART* p_part_ref = search_part_id(id);
	
	if(p_part_ref)
	{
		double new_rayon = (p_part_ref->cercle.rayon)*R_PARTICULE_FACTOR;
		
		if(!(new_rayon<R_PARTICULE_MIN))
		{
			
			double new_energy = (p_part_ref->energy)*E_PARTICULE_FACTOR;
			double x_ref = p_part_ref->cercle.centre.x;
			double y_ref = p_part_ref->cercle.centre.y;
			
			int emplacement = HAUT_G;
			for(int i=0; i<4; i++, emplacement++)
			{
				PART *new_part;
				new_part = malloc(sizeof(PART));
				new_part->energy = new_energy;
				new_part->cercle.rayon = new_rayon;
				switch(emplacement)
				{
					case HAUT_G:
						new_part->cercle.centre.x = x_ref - new_rayon;
						new_part->cercle.centre.y = y_ref + new_rayon;
						break;
					case HAUT_D:
						new_part->cercle.centre.x = x_ref + new_rayon;
						new_part->cercle.centre.y = y_ref + new_rayon;
						break;
					case BAS_G:
						new_part->cercle.centre.x = x_ref + new_rayon;
						new_part->cercle.centre.y = y_ref - new_rayon;
						break;
					case BAS_D:
						new_part->cercle.centre.x = x_ref - new_rayon;
						new_part->cercle.centre.y = y_ref - new_rayon;
						break;
				}
				insert_part(new_part);
			}
			retirer_part(p_part_ref);
		}
	}
}

void insert_part(PART *new_p)
{
	nb_id++;
	new_p->id = nb_id;
	
	double rayon = new_p->cercle.rayon;
	if(p_list_part == NULL)
	{
		new_p->suivant = NULL;
		p_list_part = new_p;
	}
	
	else if(p_list_part->cercle.rayon < rayon)
	{
		new_p->suivant = p_list_part;
		p_list_part = new_p;
	}
	
	else
	{
		PART *search=p_list_part->suivant, *prev=p_list_part;
		while(search && search->cercle.rayon > rayon)
		{
			prev = search;
			search = search->suivant;
		}
		new_p->suivant = prev->suivant;
		prev->suivant = new_p ;
	}
	
	nb_parts++;
}

void retirer_part(PART *p_part) 
{
	if(p_part == p_list_part)
	{
		p_list_part=p_list_part->suivant;
		free(p_part);
	}
	else
	{
		PART* precedent = p_list_part;
		PART* suivant = p_list_part->suivant;
		while(suivant != p_part && suivant->suivant!=NULL)
		{
			precedent = suivant;
			suivant=suivant->suivant;
		}
		if(suivant == p_part)
		{
			precedent->suivant=suivant->suivant;
			free(suivant);
		}
	}
	if(nb_parts!=0)
		nb_parts--;
} 

void vider_liste() 
{ 
   while(p_list_part != NULL) 
   {
		retirer_part (p_list_part);
   }
} 
