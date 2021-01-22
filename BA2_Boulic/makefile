# Makefile

all :

	gcc -Wall -Wextra -c main.cpp -o main.o
	gcc -Wall -Wextra -c utilitaire.c -o utilitaire.o
	gcc -Wall -Wextra -c graphic.c -o graphic.o
	gcc -Wall -Wextra -c error.c -o error.o
	gcc -Wall -Wextra -c simulation.c -o simulation.o
	gcc -Wall -Wextra -c robot.c -o robot.o
	gcc -Wall -Wextra -c particule.c -o particule.o
	gcc main.o error.o simulation.o robot.o particule.o graphic.o utilitaire.o -lm -lstdc++ -lglut -lGL -lGLU -lglui -Llib -L/usr/X11R6/lib -lX11 -lXext -lXmu -lXi -o projet.x
	rm -rf *.o 
	

