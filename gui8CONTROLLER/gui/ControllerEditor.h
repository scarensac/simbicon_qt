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

#include "InteractiveWorld.h"
#include "CurveEditor.h"
#include "qtmessenger.h"


/**
  * This class is used to build ControllerFramework and use it to control articulated figures.
  */
class ControllerEditor : public InteractiveWorld {
public:
    QtMessenger signal_messenger;

protected:
	//this is the physical world that contains all the objects inside
	SimBiConFramework* conF;

	//this array will be used to save/load the state of the dynamic world
	DynamicArray<double> worldState;

	//this is the name of the input file
	char inputFile[100];

	//this is the initial state of the framework...
	SimBiConFrameworkState conState;

	// This indicates the number of the future control shot and the maximal number of shots
	int nextControlShot;
	int maxControlShot;



	// These trajectories are reset after every cycle
	// dTrajX and vTrajX are sign-reversed on right stance cycles
	Trajectory1D dTrajX;
	Trajectory1D dTrajZ;
	Trajectory1D vTrajX;
	Trajectory1D vTrajZ;

	// Contains the FSM state index of the last simulation step
	int lastFSMState;

	/**
		This method is called whenever the controller has just taken a new step
	*/
	virtual void stepTaken();

public:
	/**
	 * Constructor.
	 */
	ControllerEditor(void);

	/**
	 * Destructor.
	 */
	virtual ~ControllerEditor(void);


	/**
		This method draws the desired target that is to be tracked.
	*/
	void drawDesiredTarget();

	// This is our associated curve editor
	DynamicArray<CurveEditor*> curveEditors;

	/**
	 * This method is used to create a physical world and load the objects in it.
	 */
	virtual void loadFramework();

	/**
	 * This method is used to create a physical world and load the objects in it.
	 */
	void loadFramework( int controlShot );

	/**
	 * This method is called whenever the window gets redrawn.
	 */
	virtual void draw(bool shadowMode = false);

	/**
	 * This method is used to draw extra stuff on the screen (such as items that need to be on the screen at all times)
	 */
	virtual void drawExtras();

	/**
	 * This method is used to restart the application.
	 */
	virtual void restart();

    /**
     * This method is used to put the character in the init pos.
     */
    virtual void stand();

	/**
	 * This method is used to reload the application.
	 */
	virtual void reload();

	/**
		This method is used to undo the last operation
	*/
	virtual void undo();

	/**
		This method is used to redo the last operation
	*/
	virtual void redo();

	/**
     *	This method is used when a mouse event gets generated. This method returns true if the message gets processed, false otherwise.
	 */
	virtual bool onMouseEvent(int eventType, int button, int mouseX, int mouseY);


	/**
	 * Delete all curve editors
	 */
	inline void clearEditedCurves() {
		for( uint i = 0; i < curveEditors.size(); ++i )
			delete curveEditors[i];
		curveEditors.clear();
	}

	/**
	 * Selects the trajectory to edit
	 */
    inline void addEditedCurve(Trajectory1D* editedCurve , Vector3d axis= Vector3d(0,0,0));


	inline SimBiConFramework* getFramework(){
		return conF;
	}

	/**
	 * This method gets called when the application gets initialized. 
	 */
	virtual void init();

	/**
	 * This method returns the target that the camera should be looking at
	 */
	Point3d getCameraTarget();

	/**
	* This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
	* (i.e. run simulations, and so on).
	*/
	virtual void processTask();


	/**
     * This method is to be implemented by classes extending this one. The output of this function is a point that
	 * represents the world-coordinate position of the dodge ball, when the position in the throw interface is (x, y).
	 */
	virtual void getDodgeBallPosAndVel(double x, double y, double strength, Point3d* pos, Vector3d* vel);

    virtual void apply_perturbation_on_pelvis(Vector3d f);
    virtual void apply_perturbation(Vector3d perturbation=Vector3d(0,0,0), bool new_perturbation=false);

	/*
	this function is a quick and easy way to save the current controler and the current position
	the to boolean are here to help choose which one is saved
	*/
    void save(bool save_controller=true, bool save_position=true);

    int trajectoryToEdit(int idx);

    std::vector<std::string> get_trajectory_name_list();

    /**
    * @brief handle_evolution
    * @param count_step the number of steps
    * @param phi the current phase
    * @return the evaluation result. Return -1 if the evaluation is not finished and we must continue
    */
    double handle_evolution(double phi, int count_step, bool& saving_possible);

private:
    //all those members are specials
    //they are old static variables in functions but I could not find what was the problem with them
    //so I put them here so I'm ensured they are reinitialized between each evaluations ...
    //ControllerEditor::processTask
    uint count_step;

    //ControllerEditor::handle_evolution
    double last_phi;
    int count_eval_iter;
    std::vector<double> vect_speed_z;
    std::vector<double> vect_speed_x;
    double cumul_time ;
    double sum_eval_whole_body_torques;
    double sum_eval_liquid_drag ;
    double cur_step_eval_weighted_acc;
    double sum_eval_weighted_acc ;
    bool first_pass_weighted_acc ;
    std::vector<Vector3d> vect_ang_speed_weighted_acc;
    std::vector<Vector3d> vect_ang_speed_desired_pose_weighted_acc;
    double sum_eval_stable_head;
    double sum_eval_stable_hands;
    double sum_eval_stable_pelvis;
    int eval_push_phase;
    Vector3d old_vel_head;
    Vector3d old_vel_pelvis;
    Vector3d old_angular_vel_pelvis;
    int old_count_step;

    //ControlerEditor::apply_perturbation
    Vector3d perturbation_force;
    double time_since_application;


    /**
     * @brief this function reset all the members from the old static variables that are used in the evaluaion function
     */
    void reset_members_for_evaluation();


};




