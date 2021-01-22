/*------------------------------------------------------------------------*/
/*                          fichier en-tete (graphic.h)                   */
/*------------------------------------------------------------------------*/

#ifndef GRAPHIC_H
#define GRAPHIC_H

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
/* GRAPHIC_EMPTY).                                              */
/*=================================================================*/

//Fonctions permettant le dessin de motifs sur les particules
void motif_particule(float xc, float yc, float r);
void motif_1(float xc, float yc, float r);
void motif_2(float xc, float yc, float r);
void motif_3(float xc, float yc, float r);

//Dessine un segment
void graphic_draw_segment (float x1, float y1, float x2, float y2);

//Dessine un rectangle
void graphic_draw_rectangle (float xc,float yc,float width,float height,int filled);

//Dessine un cercle
void graphic_draw_circle (float xc,float yc,float r,int filled);

//Fonction pour definir la couleur avec laquelle on dessine
void graphic_set_color3f(float r,float g, float b);

//Fonction pour determiner l'epaisseur d'une ligne
void graphic_set_line_width(float width);


#endif
