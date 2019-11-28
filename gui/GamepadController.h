#ifndef _GAMEPAD_CONTROLLER_H_
#define _GAMEPAD_CONTROLLER_H_

#include <windows.h>
#include <XInput.h>

class GamepadController {
public:

	/* enum types : buttons, triggers et stricks */
	enum BUTTON {DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, START, BACK, LEFT_THUMB, RIGHT_THUMB, LEFT_SHOULDER, RIGHT_SHOULDER, A, B, X, Y};
	enum TRIGGER {LEFT_TRIGGER, RIGHT_TRIGGER};
	enum STICK {LEFT_STICK_X, LEFT_STICK_Y, RIGHT_STICK_X, RIGHT_STICK_Y};

	/* Constructor (empty) */
    GamepadController() {}

	/* Update the state of the gamepad, return true if a gamepad is connected, false otherwise */
	bool updateState();

	/* Check if the button given in parameter is pressed (false if gamepad not connected) */
	bool isKeyPressed(BUTTON button);

	/* Return the magnitude [0,1] of the pressed trigger in parameter (0 if gamepad not connected) */
	float triggerValue(TRIGGER trigger);

	/* Return the position [-1,1] of the stick in parameter (neg. for left and down, pos. for right and up) (0 if gamepad not connected) */
	float positionStick(STICK strick);

	/* Start the vibration of the gamepad. Values in parameters are in [0,1] for left and right motor (no effect if gamepad not connected) */
    void startVibrate(float leftVal = 0, float rightVal = 0);

	/* Stop the vibration of the gamepad (no effect if gamepad not connected) */
	void stopVibrate();

private:
	XINPUT_STATE _controllerState;  // gamepad state

};

#endif
