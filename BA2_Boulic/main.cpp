/**
 * \file		main.cpp
 * \version		4.0
 * \date		2017-05-20
 * \author		Sélim Kamal and Yann Boudigou 288567 / 284597
 * \brief		main qui gère la configuration de l'interface et la fenêtre de display
 */

#include <GL/glut.h>
#include <GL/glui.h>
#include <stdbool.h>

extern "C"
{
	#include "simulation.h"
	#include "constantes.h"
}


namespace
{  
    static GLUI_EditText *edittext;
    static char rate[MAX_LINE],turn[MAX_LINE],translation[MAX_LINE],rotation[MAX_LINE];
    static char file_name[MAX_LINE] = ".txt"; 
    static char save_file[MAX_LINE] = "Save.txt";
    static char text_error[MAX_LINE] = "Error ";
    static char text_draw[MAX_LINE] = "Draw ";
    static int main_window; 
    static int record, control_type;
    static float rate_value=0, translation_value=0, rotation_value=0;
    static int turn_value=0;
    static GLfloat aspect_ratio = 1.f ;
    static GLfloat x_min, x_max, y_min, y_max;
    static bool draw;
    static bool simulation;
    static int width, height;
    
    static GLUI_StaticText *g_turn_text;
    static GLUI_StaticText *g_rate_text;
    static GLUI_StaticText *g_translation_text;
    static GLUI_StaticText *g_rotation_text;
    static GLUI_Button *start_button;
    static GLUI_Checkbox *g_record;
    
    FILE* p_fichier_record = NULL;
    
}

#define MODE               1
#define NOM_FICHIER        2
#define NB_PARAMETRES      3
#define TAILLE_FENETRE     700
#define POSITION_FENETRE   200
#define POSITION_INTERFACE 900
#define MAX_RATE           100

/********** User IDs pour les fonctions callback ********/
#define BUTTON_OPEN        1
#define TEXT_OPEN          2
#define BUTTON_SAVE        3
#define TEXT_SAVE          4
#define START              5
#define STEP               6
#define RECORD             7

void simulation_function()
{
	if(!simulation)
		{
			simulation=true;
			start_button->set_name("Stop");
		}
	else
	{
		simulation=false;
		start_button->set_name("Start");
		
		if(p_fichier_record)
		{
			fclose(p_fichier_record);
			p_fichier_record = NULL;
		}
		g_record->set_int_val(0);
	}
}

void simulation_step()
{
	rate_value = simulation_deplacement(translation_value, rotation_value);
	sprintf(rate, "Rate: %.3f",rate_value);
	g_rate_text->set_text(rate);
	turn_value++;
	sprintf(turn, "Turn: %d",turn_value);
	g_turn_text->set_text(turn);
	
	if(record)
	{
		fprintf(p_fichier_record, "%d %.3f\n", turn_value, rate_value);
	}
	
	
	glutPostRedisplay();
}

void record_function()
{
	if(record)
	{
		if(!p_fichier_record)
			p_fichier_record = fopen("out.dat", "w");
	}
	else
	{
		if(p_fichier_record)
		{
			fclose(p_fichier_record);
			p_fichier_record = NULL;
		}
	}
}		

void control_cb(int id)
{
	switch (id)
	{
		case BUTTON_OPEN:
			if(p_fichier_record)
			{
				fclose(p_fichier_record);
				p_fichier_record = NULL;
			}
			g_record->set_int_val(0);
			
			translation_value=0;
			sprintf(translation, "Translation: %.3f",translation_value);
			g_translation_text->set_text(translation);
			
			rotation_value=0;
			sprintf(rotation, "Rotation:     %.3f",rotation_value);
			g_rotation_text->set_text(rotation);
			
			simulation=false;
			start_button->set_name("Start");
			
			rate_value=0;
			sprintf(rate, "Rate: %.3f",rate_value);
			g_rate_text->set_text(rate);
			
			turn_value=0;
			sprintf(turn, "Turn: %d",turn_value);
			g_turn_text->set_text(turn);
			
			simulation_lecture(file_name);
			if(draw)
				glutPostRedisplay();
			break;
		case BUTTON_SAVE:
			save(save_file);
			break;
		case START:
			simulation_function();
			break;
		case STEP:
			if(rate_value != MAX_RATE)
				simulation_step();
			break;
		case RECORD:
			record_function();
			break;
	}
}

void affichage(void)
{
    GLfloat gauche= -DMAX, droite = DMAX, bas= -DMAX, haut= DMAX;
    
    glClear(GL_COLOR_BUFFER_BIT);
 
    glLoadIdentity();
    
    if (aspect_ratio <= 1.)
		glOrtho(gauche, droite, bas/aspect_ratio, haut/aspect_ratio, -1.0, 1.0);
    else 
		glOrtho(gauche*aspect_ratio, droite*aspect_ratio, bas, haut, -1.0, 1.0);
    
    if(draw)
		simulation_dessin();

    glutSwapBuffers();           
}
  
void reshape(int w, int h)
{
	width = w; 
    height = h; 
  
    glViewport(0, 0, w, h); 
    
    aspect_ratio = (GLfloat)w / (GLfloat)h ; 
    
    if (aspect_ratio <= 1.) 
    {   
		x_min = -DMAX; 
		x_max = DMAX;     
		y_min = -DMAX/aspect_ratio; 
		y_max = DMAX/aspect_ratio; 
    }   
    else 
    {   
		x_min = -DMAX*aspect_ratio; 
		x_max = DMAX*aspect_ratio; 
		y_min = -DMAX; 
		y_max = DMAX; 
    }   
} 

bool compare_strings(char string_a[], char string_b[])
{
	int i = 0;
	while(string_a[i] != '\0' && string_b[i] != '\0')
	{
		if(string_a[i] != string_b[i])
			return false;
		i++;	
	}
	return true;
}

void colonne_1(GLUI *glui)
{
	GLUI_Panel *panel_open = glui->add_panel(  "Openning" ); 
	edittext = glui->add_edittext_to_panel(panel_open, (char*)"File name:", 
	                                       GLUI_EDITTEXT_TEXT, file_name,TEXT_OPEN, 
	                                       control_cb);
	glui->add_button_to_panel( panel_open,  "Open",BUTTON_OPEN, control_cb);
	
	GLUI_Panel *panel_save = glui->add_panel(  "Saving" ); 
	edittext = glui->add_edittext_to_panel(panel_save, (char*)"File name:", 
	                                       GLUI_EDITTEXT_TEXT, save_file,TEXT_SAVE,
	                                       control_cb);
	glui->add_button_to_panel( panel_save,  "Save",BUTTON_SAVE, control_cb);
	
	glui->add_column(true);
}  

void colonne_2(GLUI *glui)
{
	GLUI_Panel *panel_simulation = glui->add_panel(  "Simulation" ); 
	start_button = glui->add_button_to_panel(panel_simulation,"Start",START,
	                                         control_cb);
	glui->add_button_to_panel( panel_simulation,  "Step",STEP, control_cb);
		
	GLUI_Panel *panel_recording = glui->add_panel(  "Recording" );
	g_record = glui->add_checkbox_to_panel( panel_recording, "Record" , &record,
											RECORD, control_cb);
	
	sprintf(rate, "Rate: %.3f",rate_value);
	g_rate_text = glui->add_statictext_to_panel(panel_recording,rate); 
	
	sprintf(turn, "Turn: %d",turn_value);
	g_turn_text = glui->add_statictext_to_panel(panel_recording,turn);
		
	glui->add_column(true);
}

void colonne_3(GLUI *glui)
{
	GLUI_Panel *panel_control = glui->add_panel(  "Control mode" );
	GLUI_RadioGroup *control_mode =  glui->add_radiogroup_to_panel(panel_control,
	                                                               &control_type); 
	glui->add_radiobutton_to_group( control_mode, "Automatic" ); 
	glui->add_radiobutton_to_group( control_mode, "Manual" );
	
	GLUI_Panel *panel_robot_control = glui->add_panel(  "Robot control" );
	sprintf(translation, "Translation: %.3f",translation_value);
	g_translation_text = glui->add_statictext_to_panel(panel_robot_control,
	                                                   translation);
	sprintf(rotation, "Rotation:     %.3f",rotation_value);
	g_rotation_text = glui->add_statictext_to_panel(panel_robot_control,rotation);
	
	glui->add_button( "Exit",0 ,(GLUI_Update_CB)exit ); 
	
}

void myIdle()
{
	if(!control_type)
		deselection_robots();
	
	if(glutGetWindow() != main_window)
		glutSetWindow(main_window);
		
	if(simulation && rate_value != MAX_RATE)
		simulation_step();
		
	if(rate_value == MAX_RATE && p_fichier_record)
	{
		fclose(p_fichier_record);
		p_fichier_record = NULL;
		g_record->set_int_val(0);
	}
}

void selection_cb(int button, int state, int x, int y)
{
	if(control_type && button==GLUT_LEFT_BUTTON  && state==GLUT_UP)
	{
		translation_value=0;
		rotation_value=0;
		
		float x_coord = ((float)x/width)*(x_max - x_min) + x_min; 
        float y_coord = ((float)(height - y)/height)*(y_max - y_min) + y_min ; 
		
		simulation_select(x_coord,y_coord);
		
		sprintf(translation, "Translation: %.3f",translation_value);
		g_translation_text->set_text(translation);
		sprintf(rotation, "Rotation:     %.3f",rotation_value);
		g_rotation_text->set_text(rotation);
		
		glutPostRedisplay();
	}
}

void fleches(int key, int x, int y)
{	
	if(control_type)
	{
		switch(key)
		{
			case GLUT_KEY_UP:
				if(translation_value+DELTA_VTRAN <= VTRAN_MAX)
				{
					translation_value += DELTA_VTRAN; 
					sprintf(translation, "Translation: %.3f",translation_value);
					g_translation_text->set_text(translation);
				}
				break;
			case GLUT_KEY_DOWN:
				if(translation_value-DELTA_VTRAN >= -VTRAN_MAX)
				{
					translation_value -= DELTA_VTRAN; 
					sprintf(translation, "Translation: %.3f",translation_value);
					g_translation_text->set_text(translation);
				}
				break;
			case GLUT_KEY_LEFT:
				if(rotation_value-DELTA_VROT >= -VROT_MAX)
				{
					rotation_value -= DELTA_VROT; 
					sprintf(rotation, "Rotation:     %.3f",rotation_value);
					g_rotation_text->set_text(rotation);
				}
				break;
			case GLUT_KEY_RIGHT:
				if(rotation_value+DELTA_VROT <= VROT_MAX)
				{
					rotation_value += DELTA_VROT; 
					sprintf(rotation, "Rotation:     %.3f",rotation_value);
					g_rotation_text->set_text(rotation);
				}
				break;
		}
	}
}

int main(int argc, char *argv[])
{	
	if(p_fichier_record)
		fclose(p_fichier_record);
				
	p_fichier_record = fopen("out.dat", "w");
	
	if (argc == NB_PARAMETRES)
	{
		if (compare_strings(argv[MODE], text_error))
		{
			draw = false;
			simulation_lecture(argv[NOM_FICHIER]);
		}
		if (compare_strings(argv[MODE], text_draw))
		{
			draw = true;
		}
	}
	else
		draw = true;
			
	if (draw)
	{	
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
		glutInitWindowPosition(POSITION_FENETRE, POSITION_FENETRE);
		glutInitWindowSize(TAILLE_FENETRE, TAILLE_FENETRE);

		main_window = glutCreateWindow("Decontaminators - display");
		
		glutIdleFunc(myIdle);
		glutReshapeFunc(reshape); 
		glutDisplayFunc(affichage);
		glutMouseFunc(selection_cb);
		glutSpecialFunc(fleches);
		
		glClearColor(1.0, 1.0, 1.0, 0.0);

		GLUI *glui = GLUI_Master.create_glui("Decontaminators - control",0,
		                                     POSITION_INTERFACE,POSITION_FENETRE); 
		
		colonne_1(glui);
		colonne_2(glui);
		colonne_3(glui);
		
		if (argc==NB_PARAMETRES)
			simulation_lecture(argv[NOM_FICHIER]);
		
		glui->set_main_gfx_window( main_window );
		
		glutMainLoop();
	}
}

