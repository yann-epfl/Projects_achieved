/**
 * \file		utilitaire.c
 * \version		2.1
 * \date		2017-05-03
 * \author		Sélim Kamal and Yann Boudigou 288567 / 284597
 * \brief		module de bas niveau
 */

#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <complex.h>
#include <stdio.h>

#include "graphic.h"

#include "utilitaire.h"

#define ESPACE              32
#define TAB                 9
#define VERT                0.475, 0.537, 0.2
#define JAUNE               1, 0.84, 0
#define ROUGE               1, 0, 0
#define R_ROBOT_INTERNE     0.15
#define R_PARTICULE_INTERNE	0.2
#define R_ROBOT_EXTERNE		0.5
#define TAILLE_CADRE		40

double util_distance(S2D a, S2D b)
{
	double difference_x = fabs(a.x-b.x), difference_y = fabs(a.y-b.y);
	double distance = sqrt(difference_x*difference_x+difference_y*difference_y);
	
	return distance;
}

double util_angle(S2D a, S2D b)
{
	double re = (b.x-a.x), im = (b.y-a.y);
	complex double z=re+im*I;

	double angle;
	angle = carg(z);
	
	return angle;
}

void util_range_angle(double * p_angle)
{
	double alpha = *p_angle;
	while(alpha>M_PI || alpha<=-M_PI)
		{
			if(alpha>0)
				alpha -= 2*M_PI;
			else
				alpha += 2*M_PI;
		}
	*p_angle = alpha;
}

bool util_point_dehors(S2D a, double max)
{
	bool dehors = false;
	
	if(a.x>max || a.x<-max)
		dehors = true;
		
	if(a.y>max || a.y<-max)
		dehors = true;
	
	return dehors;
}

bool util_alpha_dehors(double alpha)
{
	bool dehors = false;
	
	if(alpha>M_PI || alpha<-M_PI)
		dehors = true;
	
	return dehors;
}

bool util_point_dans_cercle(S2D a, C2D c)
{
	double difference_x = fabs(a.x-c.centre.x), difference_y = fabs(a.y-c.centre.y);
	double distance = sqrt(difference_x*difference_x+difference_y*difference_y);
	
	if (distance<(c.rayon-EPSIL_ZERO))
		return true;
	else
		return false;
}

bool util_collision_cercle(C2D a, C2D b, double * p_dist)
{
	*p_dist = util_distance(a.centre, b.centre);
	if((a.rayon+b.rayon)-EPSIL_ZERO > * p_dist)
		return true;
	else 
		return false;
}

S2D util_deplacement(S2D p, double alpha, double dist)
{
	double *p_alpha = &alpha;
	util_range_angle(p_alpha);
	
	double dx = cos(alpha)*dist;
	double dy = sin(alpha)*dist;
	
	p.x += dx;
	p.y += dy;
	
	return p;
}

bool util_ecart_angle(S2D a, double alpha, S2D b, double *p_ecart_angle)
{
	double distance = util_distance(a, b);
	if(distance>EPSIL_ZERO)
	{
		double angle_ab = util_angle(a, b);
		*p_ecart_angle = angle_ab-alpha;
		util_range_angle(p_ecart_angle);
		return true;
	}
	else return false;
}

bool util_alignement(S2D a, double alpha, S2D b)
{
	double ecart_angle;
	double *p_ecart_angle = &ecart_angle;
	if(util_ecart_angle(a, alpha, b, p_ecart_angle)
	   && fabs(*p_ecart_angle)<EPSIL_ALIGNEMENT)
		return true;
	else 
		return false;
}

bool util_inner_triangle(double la, double lb, double lc, double lb_new,
						 double *p_la_new)
{
	double g,p,L,delta_d,D,r,cos_beta;
	double la_new = *p_la_new;
	
	if(lb>lc)
	{
		g=lb;
		p=lc;
	}
	else
	{
		g=lc;
		p=lb;
	}
	
	delta_d=la;
	D=lb;	
	L=lc;
	r=lb_new;
	cos_beta=(delta_d*delta_d+L*L-D*D)/(2*delta_d*L);
	la_new=cos_beta*L-sqrt(cos_beta*cos_beta*L*L+r*r-L*L);
	
	*p_la_new = la_new;	
	
	if(la>EPSIL_ZERO && lc>EPSIL_ZERO && lb>=0 && lb_new<=g && lb_new>=p)
		return true;
	else
		return false;
}


//******************	fonctions de lecture de fichier		*************************//

bool commentaire(char ligne[])
{	
	int i=0;
	while(ligne[i]!='\0')
	{
		if(ligne[i]=='#')
			return true;
		if((ligne[i]>='0' && ligne[i]<='9') || ligne[i]=='F')
			return false;
		i++;
	}
	return true;
}

bool donnee(char ligne[], int *p_i)
{
	int i = *p_i;
	while((ligne[i]==ESPACE || ligne[i]==TAB || ligne[i]=='\n' || ligne[i]=='\r') 
	      && ligne[i]!='\0')
		i++;
	if(ligne[i]=='\0')
	{
		*p_i = i;
		return false;
	}
	else
	{
		*p_i = i;
		return true;
	}
}

void remplit_ligne_temp(char ligne[], int *p_i, char ligne_temp[])
{
	int i = *p_i;
	int j = 0;
	while(ligne[i]!=ESPACE && ligne[i]!=TAB && ligne[i]!='\0')
	{
		ligne_temp[j]=ligne[i];
		i++;
		j++;
	}
	ligne_temp[j]='\0';
	*p_i = i;
}

bool reste_donn(char ligne[], int i)
{
	while(ligne[i]!='\0')
	{
		if((ligne[i]>='0' && ligne[i]<='9') || ligne[i]=='-')
		{
			return true;
		}
		i++;
	}
	return false;
}

// ***********************   Fonctions de dessin    ******************************** //
void cadre()
{
	graphic_set_color3f(0, 0, 0);
    graphic_draw_rectangle (0, 0, TAILLE_CADRE, TAILLE_CADRE, GRAPHIC_EMPTY);
}

void dessin_robot (float xc,float yc, float angle)
{	
	graphic_set_color3f(VERT);
	graphic_draw_circle (xc, yc, R_ROBOT_EXTERNE, GRAPHIC_FILLED);
	
    graphic_set_color3f(0, 0, 0);
    graphic_draw_circle (xc, yc, R_ROBOT_EXTERNE, GRAPHIC_EMPTY);
    float x=cos(angle)*R_ROBOT_EXTERNE+xc, y=sin(angle)*R_ROBOT_EXTERNE+yc;
    graphic_draw_segment (xc, yc, x, y);
   
    graphic_draw_circle (xc, yc, R_ROBOT_INTERNE, GRAPHIC_FILLED);
}

void dessin_robot_selection (float xc,float yc, float angle)
{	
	graphic_set_color3f(ROUGE);
	graphic_draw_circle (xc, yc, R_ROBOT_EXTERNE, GRAPHIC_FILLED);
	
    graphic_set_color3f(0, 0, 0);
    graphic_draw_circle (xc, yc, R_ROBOT_EXTERNE, GRAPHIC_EMPTY);
    float x=cos(angle)*R_ROBOT_EXTERNE+xc, y=sin(angle)*R_ROBOT_EXTERNE+yc;
    graphic_draw_segment (xc, yc, x, y);
   
    graphic_draw_circle (xc, yc, R_ROBOT_INTERNE, GRAPHIC_FILLED);
}

void dessin_part (float xc,float yc,float r)
{ 
	graphic_set_color3f(JAUNE);
	graphic_draw_circle (xc, yc, r, GRAPHIC_FILLED);
	graphic_set_color3f(0, 0, 0);
	graphic_draw_circle (xc, yc, r, GRAPHIC_EMPTY);
	
	graphic_draw_circle (xc, yc, R_PARTICULE_INTERNE*r , GRAPHIC_FILLED);

	motif_particule(xc,yc,r);
}
