#include "GamepadController.h"
#define MAX_VIBRATION 65535
#define MAX_TRIGGER 255
#define MIN_STICK -32768
#define MAX_STICK 32767
#define DEAD_ZONE_STICK 0.2

bool GamepadController::updateState() {
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));
	// Get and check the state (return false if not connected)
	return XInputGetState(0, &_controllerState) == ERROR_SUCCESS;
}

bool GamepadController::isKeyPressed(BUTTON button) {
	if (updateState()) {
		switch (button) {
		case GamepadController::DPAD_UP: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP;
		case GamepadController::DPAD_DOWN: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN;
		case GamepadController::DPAD_LEFT: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT;
		case GamepadController::DPAD_RIGHT: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT;
		case GamepadController::START: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_START;
		case GamepadController::BACK: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_BACK;
		case GamepadController::LEFT_THUMB: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB;
		case GamepadController::RIGHT_THUMB: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB;
		case GamepadController::LEFT_SHOULDER: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER;
		case GamepadController::RIGHT_SHOULDER: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER;
		case GamepadController::A: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_A;
		case GamepadController::B: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_B;
		case GamepadController::X: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_X;
		case GamepadController::Y: return _controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_Y;
		default: return false;
		}
	}
	return false;
}

float GamepadController::triggerValue(TRIGGER trigger) {
	if (updateState()) {
		switch (trigger) {
		case GamepadController::LEFT_TRIGGER: return ((float)_controllerState.Gamepad.bLeftTrigger) / MAX_TRIGGER;
		case GamepadController::RIGHT_TRIGGER: return ((float)_controllerState.Gamepad.bRightTrigger) / MAX_TRIGGER;
		default: return 0.0;
		}
	}
	return 0.0;
}

float GamepadController::positionStick(STICK strick) {
	if (updateState()) {
		float pos = 0.0;
		// Get the raw position
		switch (strick) {
		case GamepadController::LEFT_STICK_X: pos = (float)_controllerState.Gamepad.sThumbLX; break;
		case GamepadController::LEFT_STICK_Y: pos = (float)_controllerState.Gamepad.sThumbLY; break;
		case GamepadController::RIGHT_STICK_X: pos = (float)_controllerState.Gamepad.sThumbRX; break;
		case GamepadController::RIGHT_STICK_Y: pos = (float)_controllerState.Gamepad.sThumbRY; break;
		default: break;
		}
		// Normalize to [-1,1]
		pos = pos / ((pos<0) ? -MIN_STICK : MAX_STICK);
		// Zeroise in deadzone
		if (pos < 0 && pos > -DEAD_ZONE_STICK) pos = 0.0;
		if (pos > 0 && pos < DEAD_ZONE_STICK) pos = 0.0;
		// Renormalize value to [-1,1]
		if (pos > 0) pos = (pos - DEAD_ZONE_STICK) / (1 - DEAD_ZONE_STICK);
		if (pos < 0) pos = (pos + DEAD_ZONE_STICK) / (1 - DEAD_ZONE_STICK);
		// Return the position
		return pos;
	}
	return 0.0; // return 0 if not connected
}

void GamepadController::startVibrate(float leftVal, float rightVal) {
	// Create a Vibration State
	XINPUT_VIBRATION Vibration;
	// Zeroise the Vibration
	ZeroMemory(&Vibration, sizeof(XINPUT_VIBRATION));
	// Set the Vibration Values
	Vibration.wLeftMotorSpeed = (unsigned short)leftVal*MAX_VIBRATION;
	Vibration.wRightMotorSpeed = (unsigned short)rightVal*MAX_VIBRATION;
	// Vibrate the controller
	XInputSetState(0, &Vibration);
}

void GamepadController::stopVibrate() {startVibrate();}

