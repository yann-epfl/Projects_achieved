#ifndef CONSTANTES_H
#define CONSTANTES_H

#include "tolerance.h"

#define DELTA_T				0.25
#define VTRAN_MAX			0.75
#define VROT_MAX			0.5
#define DELTA_VROT			0.125
#define DELTA_VTRAN			0.25
#define DMAX 				20
#define R_ROBOT				0.5
#define R_PARTICULE_MAX		4
#define R_PARTICULE_MIN		0.3
#define R_PARTICULE_FACTOR	0.4142
#define E_PARTICULE_MAX 	1
#define E_PARTICULE_FACTOR	0.25
#define DECOMPOSITION_RATE	0.025
#define MAX_LINE 			120

//simulation.c, robot.c et particule.c
#define ROB 1
#define PARTICULE 2

//robot.c et particule.c
#define FIN_LISTE           0
#define NB_REF              1
#define DONNEES             2
#define LG_MAX_COORD        10

//objectif simulation.c et robot.c
#define LOIN 				0
#define COLLISION 			1
#define ALIGNE 				2

#endif
