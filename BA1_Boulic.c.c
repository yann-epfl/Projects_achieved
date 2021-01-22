/**
 * \file		284597.c / nom à modifier pour le rendu suivant (section 4)
 * \version		5.4
 * \date		2017-12-02
 * \author		Boudigou 284597
 * \brief		Rendu 2 pour le projet "Me too ?" du cours CS-111(c) - 2017
 */

// *******************************************************************
// 		inclusion de fichiers en-tête avec la directives include

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

// *******************************************************************
//			   Symboles définis avec la directive define

#define NB_SIM_MIN 		1
#define N_MIN 			2
#define NB_PERS_MIN 	2
#define MAX_CYCLES 		200
#define INDICE_PERS_INIT_CONTAM 0
#define INDICE_PREMIERE_PERS_A_VACC 1

//             Symboles définies avec l'instruction enum

//   L veut dire "ligne" et C veut dire "colonne"

enum{POS_L, POS_C, BUT_L, BUT_C, COL_STATUT, NB_INFOS_PERS};
enum{NON_CONTAMINE, VACCINE, PORTEUR, CONTAMINE};

// *******************************************************************
//						Functions declaration

// ########   ne pas modifier ces déclarations de fonctions    #######

/**
 * \brief	Si nbSim n'est pas strictement positif
 * \param   nbSim		la valeur invalide de nbSim
 */
static void erreur_nbSim(int nbSim);

/**
 * \brief	Si la taille du monde n'est pas strictement supérieure à 1
 * \param 	n	   		la valeur invalide
 */
static void erreur_taille_monde(int n);

/**
 * \brief	Si le nombre de personnes n'est pas dans [2, n*n]
 * \param 	nbP		la valeur invalide
 * \param 	n		la taille du monde
 */

static void erreur_nbP(int nbP, int n);

/**
 * \brief	Si un indice de position ou de but n'est pas dans [0, n-1]
 * \param 	indice	             la première valeur d'indice invalide
 * \param 	indicePersonne		 l'indice de la personne dans [0, nbP-1]
 */

static void erreur_indice_ligne_colonne(int indice, int indicePersonne);

/**
 * \brief	Si deux personnes ont la même position
 * \param 	indiceP_A	l'indice de la premiere personne dans [0, nbP-1]
 * \param 	indiceP_B	l'indice de la seconde  personne dans [0, nbP-1]
 */
static void erreur_superposition(int indiceP_A, int indiceP_B);

// ############################ END ##################################

static void lire_verbose(void);
static bool lire_afficher_grille(void);
static int  lire_nb_sim(void);
static int  lire_taille_monde(void);
static int  lire_nb_personnes(int n);
static void init_tab_personnes_ref(int n, int nb_pers, 
								   int tab_pers_ref[][NB_INFOS_PERS]);
static void init_tab_pers(int nb_pers, int tab_pers_ref[][NB_INFOS_PERS],
						 int tab_pers[][NB_INFOS_PERS], int nb_pers_non_vaccine);
static void affichage_monde(int nb_pers, int tab_pers[][NB_INFOS_PERS],
							int n, char monde[][n]);
static void maj_monde_globale(int nb_pers, int tab_pers[][NB_INFOS_PERS],
					   int n, char monde[][n]);
static void maj_monde_locale(int indice_pers_ref, int nb_pers, int n, char monde[][n]
								  ,int tab_pers[][NB_INFOS_PERS]);
static void deplacement(int i, int n, int nb_pers,
						int tab_pers[][NB_INFOS_PERS], char monde[][n]);
static int rebouclement(int n, int coordonnee);
static int but_atteint(int i, int tab_pers[][NB_INFOS_PERS]);
static void maj_but(int i, int n, int tab_pers[][NB_INFOS_PERS]);
static int test_bloquage(int i, int nb_pers, int n, char monde[][n],
						 int tab_pers[][NB_INFOS_PERS], int x_depl, int y_depl);
static int test_bloquage_complet (int indice_pers, int nb_pers, int n, char monde[][n]
								  ,int tab_pers[][NB_INFOS_PERS]);
static int conversion_pos_pers(int indice_lgn, int indice_col, int nb_pers,
							   int tab_pers [][NB_INFOS_PERS]);
static void test_contamination(int indice_pers, int nb_pers, int n, char monde[][n]
								  ,int tab_pers[][NB_INFOS_PERS]);
static void proliferation(int nb_pers, int tab_pers[][NB_INFOS_PERS]);
static int verif_contam(int nb_pers, int tab_pers[][NB_INFOS_PERS],
						int nb_pers_non_vaccine);
static int simulation(bool afficher_grille, int nb_sim, int n, int nb_pers_non_vaccine,
					  int nb_pers, int tab_pers_ref[][NB_INFOS_PERS]);
static float calc_val_mediane(int nb_sim, int resultats[]);
static void afficher_resultats(int nb_pers, int n, int nb_pers_non_vaccine,
								int nb_sim, int resultats []);

// *******************************************************************
//						Global variable

/** variable booléenne déterminant l'affichage des messages d'invitation */
static bool	verbose;

// *******************************************************************
//						MAIN

int main(void)
{
   // lecture des paramètres généraux du programme
	lire_verbose();
	bool afficher_grille = lire_afficher_grille();
	
	int nb_sim = lire_nb_sim();
	int n = lire_taille_monde();
	int nb_pers = lire_nb_personnes(n);

    // lecture et test des descriptions des personnes
	int tab_pers_ref[nb_pers][NB_INFOS_PERS];
	init_tab_personnes_ref( n, nb_pers, tab_pers_ref);
	
	// tableau des résultats des simulations pour des paramètres identiques
	int resultats[nb_sim];
	
	// lancement des simulations
	
	// boucle de densite
	for (; nb_pers>=NB_PERS_MIN; nb_pers--)
	{
		int nb_pers_non_vaccine = nb_pers;
		// boucle de vaccination
		for (; nb_pers_non_vaccine>=NB_PERS_MIN; nb_pers_non_vaccine--)
		{
			// boucle de simulation
			for (int i=0; i<nb_sim; i++)
			{
				resultats[i] = simulation(afficher_grille, nb_sim, n,
										  nb_pers_non_vaccine, nb_pers, tab_pers_ref);
				if(verbose)
					printf("# A simulation for a context has ended !\n");
			}
			if(verbose)
				printf("# All simulations for a context have ended !\n");
			afficher_resultats(nb_pers, n, nb_pers_non_vaccine, nb_sim, resultats);
		}
		if(verbose)
			printf("# All contexts for one density have ended !\n");
		printf("\n");
	}
	
	if(verbose)
		printf("# All simulations for all contexts have ended !\n");
	return EXIT_SUCCESS;
}

static void lire_verbose(void)
{
	int dump = 0;
	scanf("%d", &dump);
	verbose = (dump != 0);
}

static bool lire_afficher_grille(void)
{
	int dump = 0;
	if(verbose)
		printf("# Do you want to see the grid ?\n");
	scanf("%d", &dump);
	return dump;
}

static int lire_nb_sim(void)
{
	int nb_sim;
	if(verbose)
		printf("# How many simulations ?\n");
	scanf("%d", &nb_sim);
	if(nb_sim < NB_SIM_MIN)
	{
		erreur_nbSim(nb_sim);
		exit(EXIT_FAILURE);
	}
	return nb_sim;
}

static int lire_taille_monde(void)
{
	int n;
	if(verbose)
		printf("# How large is the world ?\n");
	scanf("%d", &n);
	if(n < N_MIN)
	{
		erreur_taille_monde(n);
		exit(EXIT_FAILURE);
	}
	return n;
}

static int lire_nb_personnes(int n)
{
	int nb_pers;
	if(verbose)
		printf("# How many people ?\n");
	scanf("%d", &nb_pers);
	if(nb_pers < NB_PERS_MIN || nb_pers > n * n)
	{
		erreur_nbP(nb_pers, n);
		exit(EXIT_FAILURE);
	}
	return nb_pers;
}

// stocke les données de position durablement (référence)
static void init_tab_personnes_ref(int n, int nb_pers, 
								   int tab_pers_ref[][NB_INFOS_PERS])
{
	int i,j,k ;
	
	for(i = 0 ; i < nb_pers ; i++)
		for(k = 0 ; k < COL_STATUT ; k++)
		{
			scanf("%d", &tab_pers_ref[i][k]);
			if(tab_pers_ref[i][k] < 0 || tab_pers_ref[i][k] >= n)
			{
				erreur_indice_ligne_colonne(tab_pers_ref[i][k], i);
				exit(EXIT_FAILURE);
			}
		}
	
	for(i = 0 ; i < nb_pers - 1 ; i++)
		for(j = i + 1 ; j < nb_pers ; j++)
			if(tab_pers_ref[i][POS_L] == tab_pers_ref[j][POS_L] &&
			   tab_pers_ref[i][POS_C] == tab_pers_ref[j][POS_C])
			{
				erreur_superposition(i, j);
				exit(EXIT_FAILURE);
			}
}

// contient les données de position qui vont varier
static void init_tab_pers(int nb_pers, int tab_pers_ref[][NB_INFOS_PERS],
						 int tab_pers[][NB_INFOS_PERS], int nb_pers_non_vaccine)
{
	// récupère les valeurs de position de référence
	int i,j;
	for(i= 0 ; i < nb_pers ; i++)
		for(j = 0 ; j < NB_INFOS_PERS ; j++)
			tab_pers[i][j] = tab_pers_ref[i][j];
	
	// initialise le statut des personnes en fonction du taux de vaccination
	for(i = 0 ; i < nb_pers ; i++)
		tab_pers[i][COL_STATUT] = NON_CONTAMINE;	
	tab_pers[INDICE_PERS_INIT_CONTAM][COL_STATUT] = CONTAMINE;
	int nb_pers_a_vacciner = nb_pers-nb_pers_non_vaccine;
	for (int i=INDICE_PREMIERE_PERS_A_VACC; i<=nb_pers_a_vacciner; i++)
		tab_pers[i][COL_STATUT]=VACCINE;
}

// si une coordonnée est en dehors du monde, la convertit en coordonée dans le monde
static int rebouclement(int n, int coordonnee)
{
	if (coordonnee >= n)
		coordonnee = 0;
	else if (coordonnee < 0)
		coordonnee = n-1;
		
	return coordonnee;
}

// affiche la grille du monde
static void affichage_monde(int nb_pers, int tab_pers[][NB_INFOS_PERS],
							int n, char monde[][n])
{
	maj_monde_globale(nb_pers, tab_pers, n, monde);
	
	for (int i=0; i<n; i++)
	{
		printf("# ");
		for (int j=0; j<n; j++)
		{
			printf("----");
		}
		printf("-\n");
		printf("# ");
		for (int j=0; j<n; j++)
		{
			printf("| %c ", monde[i][j]);
		}
		printf("|\n");
	}
	
	printf("# ");
	for (int j=0; j<n; j++)
		{
			printf("----");
		}
	printf("-\n");
	
	printf("# ");
	for (int j=0; j<n; j++)
		{
			printf("====");
		}
	printf("=\n");
}

// met entièrement à jour le monde
static void maj_monde_globale(int nb_pers, int tab_pers[][NB_INFOS_PERS],
					   int n, char monde[][n])
{
	// remplit la grille avec des espaces (réinitialisation)
	for (int i=0; i<n; i++)
	{
		for (int j=0; j<n; j++)
		{
			monde[i][j]= ' ';
		}
	}
	
	// place les personnes et les buts dans la grille
	for (int i=0; i<nb_pers; i++)
		{
			int lgn_pers = tab_pers[i][POS_L];
			int col_pers = tab_pers[i][POS_C];
			int statut = tab_pers[i][COL_STATUT];
			char symbole;
			switch (statut)
			{
				case VACCINE:
					symbole = 'V';
					break;
				case PORTEUR:
					symbole = 'N';
					break;
				case CONTAMINE:
					symbole = 'C';
					break;
				default:
					symbole = 'N';
					break;
				}
			int lgn_but = tab_pers[i][BUT_L];
			int col_but = tab_pers[i][BUT_C];
			
			monde[lgn_pers][col_pers] = symbole;
			if(monde[lgn_but][col_but] == ' ')
				monde[lgn_but][col_but] = 'b';
		}
}

// met à jour les cases du monde adjacentes à la personne choisie
static void maj_monde_locale(int indice_pers_ref, int nb_pers, int n, char monde[][n]
								  ,int tab_pers[][NB_INFOS_PERS])
{
	int lgn_pers_ref = tab_pers[indice_pers_ref][POS_L];
	int col_pers_ref = tab_pers[indice_pers_ref][POS_C];

	// remplit les cases adjacentes d'espaces (réinitialisation locale)
	for (int i=-1; i<=1; i++)
	{
		for (int j=-1; j<=1; j++)
		{
			int lgn_case_a_maj = rebouclement(n, lgn_pers_ref+i);
			int col_case_a_maj = rebouclement(n, col_pers_ref+j);
			monde[lgn_case_a_maj][col_case_a_maj]= ' ';
		 }
	}
	
	/* place les personnes dans la grille (pas besoin de mettre les buts)
	remarque : seules les cases adjacentes seront correctes car réinitialisées */
	for (int i=0; i<nb_pers; i++)
		{
			int lgn_pers = tab_pers[i][POS_L];
			int col_pers = tab_pers[i][POS_C];
			int statut = tab_pers[i][COL_STATUT];
			char symbole;
			switch (statut)
			{
				case VACCINE:
					symbole = 'V';
					break;
				case PORTEUR:
					symbole = 'N';
					break;
				case CONTAMINE:
					symbole = 'C';
					break;
				default:
					symbole = 'N';
					break;
				}
			monde[lgn_pers][col_pers] = symbole;
		}
}

// déplace une personne selon le plus court chemin en prenant en compte le blocage
static void deplacement(int i, int n, int nb_pers,
						int tab_pers[][NB_INFOS_PERS], char monde[][n])
{
	int x_depl;
	int y_depl;
	int delta_x = tab_pers[i][BUT_L] - tab_pers[i][POS_L];
	int delta_y = tab_pers[i][BUT_C] - tab_pers[i][POS_C];

	if (fabs(delta_x)>(n/2))
		delta_x = -delta_x;

	if (delta_x == 0)
		x_depl = 0;
	else if (delta_x > 0)
		x_depl = 1;
	else x_depl = -1;

	if (fabs(delta_y)>(n/2))
		delta_y = -delta_y;

	if (delta_y == 0)
		y_depl = 0;
	else if (delta_y > 0)
		y_depl = 1;
	else y_depl = -1;

	maj_monde_locale(i, nb_pers, n, monde, tab_pers);
	
	int bloque = test_bloquage(i, nb_pers, n, monde, tab_pers, x_depl, y_depl);
	if (!bloque)
	{
		tab_pers[i][POS_L] += x_depl;
		tab_pers[i][POS_C] += y_depl;
		tab_pers[i][POS_L] = rebouclement(n, tab_pers[i][POS_L]);
		tab_pers[i][POS_C] = rebouclement(n, tab_pers[i][POS_C]);
	}
	else
	{
		int compl_bloque = test_bloquage_complet(i, nb_pers, n, monde, tab_pers);
			if (compl_bloque)
				maj_but(i, n, tab_pers);
	}
}

// détermine si la case où la personne veut se déplacer est occupée
static int test_bloquage(int i, int nb_pers, int n, char monde[][n],
						 int tab_pers[][NB_INFOS_PERS], int x_depl, int y_depl)
{	
	int bloque = 0;
	int lgn_pers_arrivee = tab_pers[i][POS_L] + x_depl;
	lgn_pers_arrivee = rebouclement(n, lgn_pers_arrivee);
	int col_pers_arrivee = tab_pers[i][POS_C] + y_depl;
	col_pers_arrivee = rebouclement(n, col_pers_arrivee);
	char case_a_verifier = monde[lgn_pers_arrivee][col_pers_arrivee];

	if (case_a_verifier != ' ')
		bloque = 1;
	return bloque;
}

// détermine si la personne est complètement bloquée
static int test_bloquage_complet (int indice_pers, int nb_pers, int n, char monde[][n],
								  int tab_pers[][NB_INFOS_PERS])
{	
	int pos_lgn_pers = tab_pers[indice_pers][POS_L];
	int pos_col_pers = tab_pers[indice_pers][POS_C];

	for (int i=-1; i<=1; i++)
	{
		for (int j=-1; j<=1; j++)
		{
			int lgn_case_a_verifier = rebouclement(n, pos_lgn_pers+i);
			int col_case_a_verifier = rebouclement(n, pos_col_pers+j);
			char case_a_verifer = monde[lgn_case_a_verifier][col_case_a_verifier];
			if (case_a_verifer == ' ')
			{
				tab_pers[indice_pers][POS_L] = lgn_case_a_verifier;
				tab_pers[indice_pers][POS_C] = col_case_a_verifier;
				return 0;
			}
		 }
	}
	return 1;
}

// détermine si le but est atteint ou non
static int but_atteint (int i, int tab_pers[][NB_INFOS_PERS])
{
	int but_atteint = 0;
	if (tab_pers[i][POS_L]==tab_pers[i][BUT_L] &&
		tab_pers[i][POS_C]==tab_pers[i][BUT_C])
		but_atteint = 1;
	return but_atteint;
}

// met à jour le but
static void maj_but(int i, int n, int tab_pers[][NB_INFOS_PERS])
{
	int l_but = tab_pers[i][BUT_L];
	int c_but = tab_pers[i][BUT_C];
	while (tab_pers[i][BUT_L] == l_but && tab_pers[i][BUT_C] == c_but)
	{
		tab_pers[i][BUT_L] = rand()%n;
		tab_pers[i][BUT_C] = rand()%n;
	}
}

// convertit des coordonnées en un indice de personne
static int conversion_pos_pers(int indice_lgn, int indice_col, int nb_pers,
							   int tab_pers [][NB_INFOS_PERS])
{
	for (int i=0; i<nb_pers; i++)
	{
		if(tab_pers [i][POS_L] == indice_lgn  && tab_pers [i][POS_C] == indice_col)
			return i;
	}
	return EXIT_FAILURE; // si position transmise incorrecte (ne doit pas arriver)
}

// contamine les personnes adjacentes ou la personne de référence si besoin
static void test_contamination (int indice_pers, int nb_pers, int n, char monde[n][n]
								  ,int tab_pers[nb_pers][NB_INFOS_PERS])
{
	maj_monde_locale(indice_pers, nb_pers, n, monde, tab_pers);
	
	int pos_lgn_pers = tab_pers[indice_pers][POS_L];
	int pos_col_pers = tab_pers[indice_pers][POS_C];
	char statut_pers = monde[pos_lgn_pers][pos_col_pers];

	for (int i=-1; i<=1; i++)
	{
		for (int j=-1; j<=1; j++)
		{
			int lgn_case_a_verifier = rebouclement(n, pos_lgn_pers+i);
			int col_case_a_verifier = rebouclement(n, pos_col_pers+j);
			char case_a_verifer = monde[lgn_case_a_verifier][col_case_a_verifier];
			if (statut_pers == 'C' && case_a_verifer == 'N')
			{
				int indice_pers_a_contam = conversion_pos_pers(lgn_case_a_verifier,
															   col_case_a_verifier,
															   nb_pers, tab_pers);
				tab_pers[indice_pers_a_contam][COL_STATUT] = PORTEUR;
			}
			if (statut_pers == 'N' && case_a_verifer == 'C')
				tab_pers[indice_pers][COL_STATUT] = PORTEUR;
		 }
	}
}

// transforme les personnes porteuses en personnes contaminées
static void proliferation(int nb_pers, int tab_pers[][NB_INFOS_PERS])
{
	for(int i=0; i<nb_pers; i++)
	{
		if(tab_pers[i][COL_STATUT] == PORTEUR)
			tab_pers[i][COL_STATUT] = CONTAMINE;
	}
}

// détermine si toutes les personnes non-contaminées ont été contaminées
static int verif_contam(int nb_pers, int tab_pers[][NB_INFOS_PERS],
						int nb_pers_non_vaccine)
{
	int nb_pers_contam = 0;
	for(int i=0; i<nb_pers; i++)
	{
		if(tab_pers[i][COL_STATUT] == CONTAMINE)
			++nb_pers_contam;
	}
	if (nb_pers_contam == nb_pers_non_vaccine)
		return 1;
	return 0;
}

// réalise une simulation pour un contexte donné
static int simulation(bool afficher_grille, int nb_sim, int n, int nb_pers_non_vaccine,
					  int nb_pers, int tab_pers_ref[nb_pers][NB_INFOS_PERS])
{
	// initialisation
	int tab_pers[nb_pers][NB_INFOS_PERS];
	init_tab_pers(nb_pers, tab_pers_ref, tab_pers, nb_pers_non_vaccine);
	char monde[n][n];
	int cycles = 0;
	
	// contamination initiale
	test_contamination(INDICE_PERS_INIT_CONTAM, nb_pers, n, monde, tab_pers);
	proliferation(nb_pers, tab_pers);
	int tout_le_monde_contam = verif_contam(nb_pers, tab_pers, nb_pers_non_vaccine);
	
	// affichage initial
	if (afficher_grille)
		affichage_monde(nb_pers, tab_pers, n, monde);
	
	// boucle générale
	while (cycles<MAX_CYCLES && !tout_le_monde_contam)
	{
		// boucle pour une personne
		for (int i = 0; i<nb_pers; i++)
		{
			if (but_atteint(i, tab_pers))
				maj_but(i, n, tab_pers);
			deplacement(i, n, nb_pers, tab_pers, monde);
			test_contamination (i, nb_pers, n, monde, tab_pers);
		}
		
		// mise à jour des conditions d'arrêt
		proliferation(nb_pers, tab_pers);
		tout_le_monde_contam = verif_contam(nb_pers, tab_pers, nb_pers_non_vaccine);
		++cycles;
		
		if (afficher_grille)
			affichage_monde(nb_pers, tab_pers, n, monde);
	}

	// retour de la durée de contamination
	int duree_contam = cycles;
	return duree_contam;
}

// calcule la valeur médiane
static float calc_val_mediane (int nb_sim, int resultats[])
{
	int i, j, jmin, min;

	for(i=0 ; i<=nb_sim/2 ; i++)
	{  
	   min  = resultats[i];
	   jmin = i;
	   for(j=i+1 ; j<nb_sim ; j++)
	   {
		  if(resultats[j]<min)
		  {
			 min = resultats[j];
			 jmin = j;
		  }
	   }
	   resultats[jmin] = resultats[i];
	   resultats[i] = min;
	}
	
	float val_mediane;
	if(nb_sim%2 == 0)
		val_mediane = (float)(resultats[(nb_sim-2)/2]+resultats[(nb_sim)/2])/2;
	else val_mediane = resultats[(nb_sim-1)/2];
	
	return val_mediane;
}

// affiche les résultats
static void afficher_resultats(int nb_pers, int n, int nb_pers_non_vaccine,
								int nb_sim, int resultats [])
{
	float densite = (float)nb_pers/(n*n);
	float taux_vaccination = (float)(nb_pers-nb_pers_non_vaccine)/nb_pers;
	float duree_contam = calc_val_mediane (nb_sim, resultats);
	printf("%f	%f	%f\n", densite, taux_vaccination, duree_contam);
}


// *******************************************************************

// ################## Ne pas modifier ces fonctions ##################
//====================================================================//
//																	  //
//				***		 Fonctions d'erreurs   	***					  //
//																	  //
//					  /!\ NE PAS MODIFIER /!\						  //
//																	  //
//====================================================================//

static void erreur_nbSim(int nbSim)
{
	printf("nbSim (=%d) ne valide pas nbSim > 0 !\n", nbSim);
}

static void erreur_taille_monde(int n)
{
	printf("n (=%d) ne valide pas n > 1 !\n", n);
}

static void erreur_nbP(int nbP, int n)
{
	printf("nbP (=%d) ne valide pas nbP > 1 ET nbP <= %d !\n", nbP, n*n);
}

static void erreur_indice_ligne_colonne(int indice, int indicePersonne)
{
	printf("indice incorrect %d de ligne ou colonne de la personne d'indice %d !\n", 
	       indice, indicePersonne);
}

static void erreur_superposition(int indiceP_A, int indiceP_B)
{
	// s'assure d'avoir les indices A et B dans l'ordre croissant
	if(indiceP_B > indiceP_A) 
	{
		int temp = indiceP_A;
		indiceP_A = indiceP_B;
		indiceP_B = temp;
	}
	
	printf("les personnes d'indices %d et %d se superposent !\n", 
		   indiceP_A, indiceP_B);
}

// ############################ END ##################################
