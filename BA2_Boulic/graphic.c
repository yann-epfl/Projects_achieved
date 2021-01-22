/**
 * \file		graphic.c
 * \version		2.0
 * \date		2017-05-20
 * \author		Kamal Selim 288567
 * \brief		module gerant les dessins d'elements uniques
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#include <GL/glu.h>

#include "constantes.h"

#include "graphic.h"

#define SIDES	            50
#define R_PETIT_ARC	        0.3
#define R_GRAND_ARC	        0.835
#define ANGLE_DESSIN_1	    M_PI/3
#define ANGLE_DESSIN_2      2*M_PI/3
#define ANGLE_DESSIN_3      4*M_PI/3
#define GRAPHIC_EMPTY       0
#define GRAPHIC_FILLED      1

/*=================================================================*/
/* Fonctions pour afficher un segment, un rectangle ou un cercle.  */
/*								   */
/* Pour le segment, on fournit les coordonnnees des deux extremites*/
/*                                                                 */
/* Pour le rectangle, on fournit les coordonnees de son centre et  */
/* de sa taille (width, height).                                   */
/*                                                                 */
/* Pour le cercle, on fournit les coordonnees de son centre et son */
/* rayon.                                                          */
/*                                                                 */
/* Le rectangle et le cercle peuvent etre creux ou pleins (filled) */
/* suivant la valeur du dernier argument (GRAPHIC_FILLED ou bien   */
/* GRAPHIC_EMPTY).                                                 */
/*=================================================================*/

void graphic_draw_segment (float x1,float y1,float x2,float y2)

{ 
    glBegin (GL_LINES);

    glVertex2f (x1, y1);
    glVertex2f (x2, y2);

    glEnd ();
}



void graphic_draw_rectangle (float xc,float yc,float width,float height,int filled)

{   
	if (filled == GRAPHIC_FILLED)
		glBegin (GL_POLYGON);
    else
		glBegin (GL_LINE_LOOP);

    glVertex2f (xc+width/2, yc+height/2);
    glVertex2f (xc-width/2, yc+height/2);
    glVertex2f (xc-width/2, yc-height/2);
    glVertex2f (xc+width/2, yc-height/2);

    glEnd ();
}

void motif_particule(float xc, float yc, float r)
{
	motif_1(xc,yc,r);
	motif_2(xc,yc,r);
	motif_3(xc,yc,r);
}

void motif_1(float xc, float yc, float r)
{
	int i;
    
	glBegin (GL_POLYGON);
  
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = i * ANGLE_DESSIN_1 / SIDES;
			float x = xc - R_GRAND_ARC*r*cos(alpha), y = yc + R_GRAND_ARC*r*sin(alpha);
			glVertex2f (x, y); 
        } 
    }
    
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = ANGLE_DESSIN_2+(i * ANGLE_DESSIN_1 / SIDES);
			float x = xc + R_PETIT_ARC*r*cos(alpha), y = yc + R_PETIT_ARC*r*sin(alpha);
			glVertex2f (x, y); 
        } 
    }

    glEnd ();
}

void motif_2(float xc, float yc, float r)
{
	int i;
    
    glBegin (GL_POLYGON);
  
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = i * ANGLE_DESSIN_1 / SIDES;
			float x = xc + R_GRAND_ARC*r*cos(alpha), y = yc + R_GRAND_ARC*r*sin(alpha);
			glVertex2f (x, y); 
        } 
    }
    
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = ANGLE_DESSIN_2+(i * ANGLE_DESSIN_1 / SIDES);
			float x = xc - R_PETIT_ARC*r*cos(alpha), y = yc + R_PETIT_ARC*r*sin(alpha);
			glVertex2f (x, y); 
        } 
    }

    glEnd ();
}

void motif_3(float xc, float yc, float r)
{
	int i;
    
    glBegin (GL_POLYGON);
  
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = ANGLE_DESSIN_3 + (i * ANGLE_DESSIN_1 / SIDES);
			float x = xc + R_GRAND_ARC*r*cos(alpha), y = yc + R_GRAND_ARC*r*sin(alpha);
			glVertex2f (x, y); 
        } 
    }
    
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = ANGLE_DESSIN_1+(i * ANGLE_DESSIN_1 / SIDES);
			float x = xc + R_PETIT_ARC*r*cos(alpha), y = yc - R_PETIT_ARC*r*sin(alpha);
			glVertex2f (x, y); 
        } 
    }

    glEnd ();
}



void graphic_draw_circle (float xc,float yc,float r,int filled)

{ 
	int i;

    if (filled == GRAPHIC_FILLED)
		glBegin (GL_POLYGON);
    else
		glBegin (GL_LINE_LOOP);
	
    for (i=0; i < SIDES; i++)
    {
        for (i=0; i < SIDES; i++) 
        {  
			float alpha = i * 2. * M_PI / SIDES;
			float x = xc + r * cos (alpha), 
			y = yc + r * sin (alpha);
			glVertex2f (x, y); 
        } 
    }

    glEnd ();
}

void graphic_set_color3f(float r, float g, float b)
{
	glColor3f(r, g, b);
}

void graphic_set_line_width(float width)
{
	glLineWidth(width);
}
