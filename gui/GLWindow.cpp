/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "GLWindow.h"

#include <GL/freeglut.h>
#include <GLUtils/GLUtils.h>
#include "Globals.h"
#include "Core/SimGlobals.h"
#include "SPlisHSPlasH/Interface.h"
#include <iostream>

GLWindow::GLWindow(int x, int y, int w, int h){
	ellapsedTime = 0;
	oldFrameRate = 0;
	nrFramesSinceUpdate = 0;
	processingTime = 0;
    oldPerformanceRate = 0;
    timeSpentProcessing = 0;
    timeSpentProcessing2 = 0;
	this->camera.target = Point3d(0,1,0);
	this->x = x;
	this->y = y;
	this->w = w;
	this->h = h;


    screenShotNumber=0;
    worldShotNumber=0;

}

GLWindow::~GLWindow(void){

}

//this method is used to set up the lights (position, direction, etc)
void GLWindow::setupLights(){
    GLfloat ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat diffuse0[] = {0.9f, 0.9f, 0.9f, 1.0f};
	GLfloat diffuse[] = {0.6f, 0.6f, 0.6f, 1.0f};
    GLfloat specular[] = {0.0f, 0.0f, 0.0f, 0.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);

	glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT2, GL_SPECULAR, specular);


    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0f);



    glEnable(GL_LIGHTING);

	GLfloat light0_position[] = { 50.0f, 10.0f, 200.0f, 0.0f };
	GLfloat light1_position[] = {200.0f, 10.0f, -200.0f, 0.0f };
	GLfloat light2_position[] = {-200.0f, 10.0f, -200.0f, 0.0f };


	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);


}

/**
 *  Initializes the openGL settings
 */
void GLWindow::setupOpenGL(){
	//setup the viewport and the perspective transformation matrix
    glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix
	gluPerspective(45.0, (double)w/h,0.1,150.0);
	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix

	// set the colour of the background, as well as other shading options
	glClearColor(1.0, 1.0, 1.0, 0);
//	glClearColor(0.0, 0.0, 0.0, 0);
	glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	glEnable(GL_BLEND);
	//turn on antialiasing
    ///TODO check this because if it's activated it will do some kid of bs on the triangles borders
    //glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_LINE_SMOOTH);
    //glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	glBlendFunc(GL_SRC_ALPHA ,GL_ONE_MINUS_SRC_ALPHA);

	setupLights();

}

/**
  * this method is used to initialize the gl window
  */
void GLWindow::init(){
	setupOpenGL();
	fpsTimer.restart();
}

/** 
  *  this method is used to compute the correct camera transformation, and prepare the modelview matrix accordingly
  */
void GLWindow::setupGLCamera(){
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	camera.applyCameraTransformations();
}

/**
  * this method is called immediately before the application takes its turn to process its task
  */
void GLWindow::onStartProcessing(){
	//to make sure we can ignore all the time that GLUT and TK/TCL need to process their messages, 
	//delays between redraw() and draw(), etc, we'll restart the timer here, and
	//record the end time of the application processing.

    start = std::chrono::high_resolution_clock::now();
//    fpsTimer.restart();
}

/**
  * and this method is called immediately after the application processed its task
  */
void GLWindow::onStopProcessing(){

    std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff=end-start;
    timeSpentProcessing=diff.count();
//    timeSpentProcessing = fpsTimer.timeEllapsed();
}

/**
  * this method is used to set the desired rotation values for the camera
  */
void GLWindow::setCameraRotation(const Vector3d& newRot){
	this->camera.rotations = newRot;
}


/**
  * and this method is used to set the target that the camera looks at
  */
void GLWindow::setCameraTarget(const Point3d& newTarget){
	this->camera.target = newTarget;
}

#define FFMPEG_RENDER
#ifdef FFMPEG_RENDER
#define USE_MULTIPLES_SHADER
#endif

/**
 *	This method is used to draw the scene.
 */
void GLWindow::draw(){
	/*if (mouseButtonPressed == MOUSE_RBUTTON){
		camera.rotations += Vector3d(0, 0.0125, 0);
	}*/
	//clear the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);


    if (Globals::use_qt_interface){


		//we will now place the camera
		setupGLCamera();
		//and set up the lights (do this so that the position is specified in world coordinates, not in camera ones).
		setupLights();

		/**
		If we want fog...
		*/
		/*
		GLfloat fogColor[4]= {0.5f, 0.5f, 0.5f, 1.0f};		// Fog Color
		glFogi(GL_FOG_MODE, GL_LINEAR);		// Fog Mode
		glFogfv(GL_FOG_COLOR, fogColor);			// Set Fog Color
		glFogf(GL_FOG_DENSITY, 0.35f);				// How Dense Will The Fog Be
		glHint(GL_FOG_HINT, GL_DONT_CARE);			// Fog Hint Value
		glFogf(GL_FOG_START, 20.0f);				// Fog Start Depth
		glFogf(GL_FOG_END, 80.0f);				// Fog End Depth
		glEnable(GL_FOG);					// Enables GL_FOG
		*/

		if (Globals::drawCubeMap)
			drawCubeMap();

		glColor3d(1, 1, 1);
		//and then draw the application stuff
        if (Globals::app){

            //draw character
            Globals::app->draw();

            if (Globals::drawGroundPlane){
                //draw the ground
                Globals::app->drawGround();

                if (Globals::drawShadows){
                    //no more texture mapping or lighting for the shadows
                    glDisable(GL_LIGHTING);
                    glDisable(GL_TEXTURE_2D);

                    drawShadows();

                    //reenable them
                    glEnable(GL_LIGHTING);
                    glEnable(GL_TEXTURE_2D);
                }
            }
        }

        //no more texture mapping or lighting needed
        glDisable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
		glColor3d(1, 1, 1);

		if (Globals::app)
			Globals::app->drawExtras();

		if (Globals::drawGlobalAxes)
			drawAxes();


		//print a screenshot if needed
		if (Globals::drawScreenShots){
			char fName[100];
			sprintf(fName, "..\\screenShots\\ss%05d.bmp", screenShotNumber);
			screenShotNumber++;
			GLUtils::saveScreenShot(fName, 0, 0, w, h);
		}

		if (Globals::drawWorldShots && Globals::app){
			char fName[100];
			sprintf(fName, "..\\worldShots\\ws%05d.obj", worldShotNumber);
			worldShotNumber++;
			FILE* fp = fopen(fName, "w");
			fprintf(fp,
				"####\n"
				"#\n"
				"# OBJ File Generated by UBC-IMAGER character animation tool\n"
				"# Frame %05d\n"
				"#\n"
				"####\n\n", worldShotNumber);

			Globals::app->renderToObjFile(fp, 0);

			fclose(fp);
		}

        if (Globals::simulateFluid){
            // Enable blending

            Interface::drawParticles(true,true,false,false);


#ifdef FFMPEG_RENDER
            static bool currently_saving=false;
            static double video_start_time=0;

            static int width = glutGet(GLUT_WINDOW_WIDTH);
            static int height = glutGet(GLUT_WINDOW_HEIGHT);

            static int* buffer = new int[width*height];
            static FILE* ffmpeg = NULL;

            if (!currently_saving){
                if (!Globals::drawFPS){
                    currently_saving=true;
                    video_start_time=SimGlobals::simu_time;

                    int framerate = (1/SimGlobals::dt) / 2;
                    std::cout<<"video framerate: "<<framerate<< std::endl;
                    std::ostringstream oss;
                    // start ffmpeg telling it to expect raw rgba 720p-60hz frames
                    // -i - tells it to read frames from stdin
                    oss << "D:\\ffmpeg-4.1.3-win64-static\\bin\\ffmpeg " <<
                        " -r "<<framerate<<" -f rawvideo -pix_fmt rgba -s "<<width<<"x"<<height<<" -i - " <<
                        "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip "<<
                           "output_"<<++Globals::video_index<<".mp4";



                    // open pipe to ffmpeg's stdin in binary write mode
                    ffmpeg = _popen(oss.str().c_str(), "wb");
                }
            }

            if (currently_saving){

                glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);

                fwrite(buffer, sizeof(int)*width*height, 1, ffmpeg);



#ifdef USE_MULTIPLES_SHADER
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

            //we will now place the camera
            setupGLCamera();
            //and set up the lights (do this so that the position is specified in world coordinates, not in camera ones).
            setupLights();

            glColor3d(1, 1, 1);
            //and then draw the application stuff
            //draw character
            Globals::app->draw();

            if (Globals::drawGroundPlane){
                //draw the ground
                Globals::app->drawGround();

                if (Globals::drawShadows){
                    //no more texture mapping or lighting for the shadows
                    glDisable(GL_LIGHTING);
                    glDisable(GL_TEXTURE_2D);

                    drawShadows();

                    //reenable them
                    glEnable(GL_LIGHTING);
                    glEnable(GL_TEXTURE_2D);
                }
            }


            //no more texture mapping or lighting needed
            glDisable(GL_LIGHTING);
            glDisable(GL_TEXTURE_2D);
            glColor3d(1, 1, 1);

            Interface::drawParticles(true,true,false,true);



            glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
            fwrite(buffer, sizeof(int)*width*height, 1, ffmpeg);

#endif
            if (SimGlobals::simu_time>(video_start_time+20)){
                Globals::drawFPS=true;
            }

             if (Globals::drawFPS){
                 currently_saving=false;
                 _pclose(ffmpeg);
                 std::cout<<"finished saving"<<std::endl;
                 exit(0);
             }
         }
#endif

        }

	}
	glColor3d(1, 1, 1);

	//wait until the required ammount of time has passed (respect the desired FPS requirement)
    if ((!Globals::evolution_mode)&&(Globals::limitfps)){
        while (fpsTimer.timeEllapsed()<1.0 / Globals::desiredFrameRate);
	}

	if (Globals::drawFPS){
        drawFPSandPerf(fpsTimer.timeEllapsed(), timeSpentProcessing);
	}
		
	fpsTimer.restart();
}

/**
  * draws the shadows, by projecting the scene on the ground. Only works for planar ground surfaces!
  */
void GLWindow::drawShadows(){
	//this is the normal to the ground surface.
	//it should be changed if the ground isn't flat... For a polygonal mesh, we might have to do this for every polygon. Not fun!
	Vector3d n = Globals::app->getGroundNormal();
	//assume that the point (0, 0, 0) always belongs to the ground. This means that in the equation of the plane, d will always be 0

	//this is the direction of the light.
	Vector3d d = Vector3d(-150,200,200);

	//we'll mark where the ground is by drawing a 1 in the stencil buffer.
	glClear(GL_STENCIL_BUFFER_BIT);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	Globals::app->drawGround();

	//now we'll render the scene, using the projection matrix, and we'll only be
	//drawing where there is a 1 in the stencil buffer. This makes sure we're only drawing
	//on top of the ground, where it is visible
    glStencilFunc(GL_LESS, 0, 0xffffffff);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

    //// Draw the shadow
    glPolygonOffset(-2.0f, -2.0f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0, 0.0, 0.0, 0.5);

    glPushMatrix();
	double dot = n.dotProductWith(d);

	//this is the projection matrix
	double mat[16] = {
		dot - n.x*d.x,		-n.x * d.y,			-n.x * d.z,			0,
		-n.y * d.x,			dot - n.y * d.y,	-n.y * d.z,			0,
		-n.z * d.x,			- n.z * d.y,		dot - n.z * d.z,	0,
			0,				     0,					0,			   dot
	};

    glMultMatrixd(mat);
	if (Globals::app)
		Globals::app->draw(true);

	glPopMatrix();

    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_STENCIL_TEST);
}


/**
  * draw the background cube map
  */
void GLWindow::drawCubeMap(){
	glColor3d(1.0,1.0,1.0);
	glPushMatrix();
	double scale = 60;
	glScaled(scale, scale, scale);
	glTranslated(0, 0.2, 0);
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

/**
  * draw the global axes
  */
void GLWindow::drawAxes(){
	glPushMatrix();
	glLoadIdentity();
	glTranslated(-3,-2.0,-6.0);
	//we still want them to rotate as the camera moves, but we want it to stay put. So we'll cheat a bit...
	glRotated(-180/PI * camera.rotations.x,1.0,0.0,0.0);
	glRotated(-180/PI * camera.rotations.y,0.0,1.0,0.0);
	glRotated(-180/PI * camera.rotations.z,0.0,0.0,1.0);
	glDisable(GL_DEPTH_TEST);
	GLUtils::drawAxes(0.5);
	glEnable(GL_DEPTH_TEST);
	glPopMatrix();
}

#include <iostream>

/**
 * draw information regarding the frame rate and performance
 */
void GLWindow::drawFPSandPerf(double timeSinceLastUpdate, double timeSpentProcessing){
	ellapsedTime += timeSinceLastUpdate;
	processingTime += timeSpentProcessing;
	nrFramesSinceUpdate++;

    if(Globals::manualFPSRefresh){
        oldFrameRate = nrFramesSinceUpdate / ellapsedTime;
        oldPerformanceRate = processingTime / nrFramesSinceUpdate;
        if (Globals::refreshFPS){
            nrFramesSinceUpdate = 0;
            ellapsedTime = 0;
            Globals::refreshFPS=false;
        }
    }else{
        //only change the numbers that we display about 3 times per second
        if (ellapsedTime>=1.0/3.0){
            oldFrameRate = nrFramesSinceUpdate / ellapsedTime;
            oldPerformanceRate = processingTime / nrFramesSinceUpdate;
            nrFramesSinceUpdate = 0;
            ellapsedTime = 0;
    //        std::cout<<"processing time: "<<processingTime<<"  "<<timeSpentProcessing2<<std::endl;
        }
    }




	glPushMatrix();
	glLoadIdentity();
    glTranslatef(-1.1f,.30f,-2.5f);

	glColor3d(0.0,0.0,0.0);
    glRasterPos2f(0.28f, 0.4f);

    gprintf("             FPS: %7.2lf  \n", oldFrameRate);
//	gprintf("FPS: %7.2lf (processing: %7.2lf %%)\n", oldFrameRate, 100 * oldPerformanceRate);

    //glTranslatef(0.0f,0.0f,-0.6f);

    glColor3d(0.0,0.0,0.0);
    //glRasterPos2f(0.6f, 0.6f);

    gprintf("avgSpeed : x:%7.3lf, z:%7.3lf \n", Globals::avg_speed.x, Globals::avg_speed.z);

    if (SimGlobals::nb_container_boxes>0){
        //glTranslatef(0.0f,0.50f,-0.6f);

        glColor3d(0.0,0.0,0.0);
        glRasterPos2f(0.6f, 0.3f);
        gprintf("nb objects actifs / nb collisions : %d / %d \n",
                SimGlobals::nb_active_objects, SimGlobals::nb_collisons_event);
    }

	glPopMatrix();
}

#include <iostream>
/**
 * This method is called whenever there is a new event to be processed
 */
int GLWindow::onMouseEvent(int mEvent, int button, int x, int y){
	Vector3d v;
	TransformationMatrix camToWorld;

	if (mEvent == MOUSE_DOWN || mEvent == MOUSE_UP)
		mouseButtonPressed = button;

	//otherwise it probably is a mouse event.
	if (mEvent == MOUSE_DOWN || mEvent == MOUSE_UP || mEvent == MOUSE_DRAG || mEvent == MOUSE_MOVE){
		//see if the application wants to process this event.
		bool processed = false;
		if (Globals::app)
			if (Globals::app->onMouseEvent(mEvent, mouseButtonPressed, x, y))
				processed = true;

		

		//if not, we'll do our own thing now
		if (processed == false){
			if (mEvent == MOUSE_DRAG){
				switch (mouseButtonPressed){
					case MOUSE_RBUTTON:
								//the translation part depends on the camera orientation, so figure it out properly
								v.x = (oldMouseX - x)/200.0;
								v.y = -(oldMouseY - y)/200.0;
								v.z = 0;

								//the target is specified in world coordinates, but it is more intuitive if
								//v above is expressed in camera coordinates, so we need to switch it up
								camToWorld.setToInverseCoordFrameTransformationOf(camera.worldToCam);
								v = camToWorld * v;

								camera.target.x += v.x;
								camera.target.y += v.y;
								camera.target.z += v.z;
								break;
					case MOUSE_LBUTTON:
								camera.rotations -= Vector3d((-oldMouseY+y)/100.0, (-oldMouseX+x)/100.0, 0);
								//std::cout << camera.rotations.x << "::" << camera.rotations.y << "::" << camera.rotations.z << std::endl;
								break;
					case MOUSE_MBUTTON:
								camera.camDistance += (oldMouseY - y)/200.0;
								break;
				}
			}
			else if (button == MOUSE_WHEEL_DOWN){
				camera.camDistance += 0.1;
			}
			else if (button == MOUSE_WHEEL_UP){
				camera.camDistance -= 0.1;
			}
		}

		oldMouseX = x;
		oldMouseY = y;

//		redraw();
		return 1;
	}
	return 0;
}


/**
 * This method is called whenever there is a new event to be processed
 */
int GLWindow::onKeyEvent(int character) {
	return Globals::app->onKeyEvent(character);
}
