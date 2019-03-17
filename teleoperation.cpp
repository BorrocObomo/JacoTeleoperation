#include "stdafx.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include "Leap.h"
#include <Windows.h>
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include <conio.h>
#include "KinovaTypes.h"

using namespace std;

HINSTANCE commandLayer_handle;

int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);
int(*MyGetCartesianPosition)(CartesianPosition &);

TrajectoryPoint pointToSend;
CartesianPosition currentCommand;
CartesianPosition currentPosition;

//cannot use namespace Leap because of conflicting Kinova and Leap controllers
class SampleListener : public Leap::Listener {
public:
	virtual void onInit(const Leap::Controller&);
	virtual void onConnect(const Leap::Controller&);
	virtual void onDisconnect(const Leap::Controller&);
	virtual void onExit(const Leap::Controller&);
	virtual void onFrame(const Leap::Controller&);
	virtual void onFocusGained(const Leap::Controller&);
	virtual void onFocusLost(const Leap::Controller&);
	virtual void onDeviceChange(const Leap::Controller&);
	virtual void onServiceConnect(const Leap::Controller&);
	virtual void onServiceDisconnect(const Leap::Controller&);
};

const string fingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const string boneNames[] = { "Metacarpal", "Proximal", "Middle", "Distal" };
const string stateNames[] = { "STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END" };

void SampleListener::onInit(const Leap::Controller& controller) {
	cout << "Initialized" << endl;
}

void SampleListener::onConnect(const Leap::Controller& controller) {
	cout << "Connected" << endl;
}

void SampleListener::onDisconnect(const Leap::Controller& controller) {
	// Note: not dispatched when running in a debugger.
	cout << "Disconnected" << endl;
}

void SampleListener::onExit(const Leap::Controller& controller) {
	cout << "Exited" << endl;
}

void SampleListener::onFrame(const Leap::Controller& controller) {

	const Leap::Frame frame = controller.frame();

	Leap::HandList hands = frame.hands();
	for (Leap::HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
		// Get the first hand
		const Leap::Hand hand = *hl;
		/*string handType = hand.isLeft() ? "Left hand" : "Right hand";
		cout << string(2, ' ') << handType << ", id: " << hand.id()
		<< ", palm position: " << hand.palmPosition() << endl;*/
		// Get the hand's normal vector and direction
		// const Leap::Vector normal = hand.palmNormal();
		// const Leap::Vector direction = hand.direction();

		//// Calculate the hand's pitch, roll, and yaw angles
		//cout << string(2, ' ') << "pitch: " << direction.pitch() * Leap::RAD_TO_DEG << " degrees, "
		//	<< "roll: " << normal.roll() * Leap::RAD_TO_DEG << " degrees, "
		//	<< "yaw: " << direction.yaw() * Leap::RAD_TO_DEG << " degrees" << endl;

		// Get the Arm bone
		Leap::Arm arm = hand.arm();
		//cout << endl << endl << endl << arm.direction().x << arm.direction().y << arm.direction().z << endl;

		cout << "Arm direction: " << arm.direction() << " Grab strength: " << hand.grabStrength() << endl;

		//Update robot data
		(*MyGetCartesianCommand)(currentCommand);
		(*MyGetCartesianPosition)(currentPosition);

		//We specify that this point will be used with a angular(joint by joint) velocity vector.
		pointToSend.Position.Type = CARTESIAN_VELOCITY;
		pointToSend.Position.CartesianPosition.X = -arm.direction().x;
		pointToSend.Position.CartesianPosition.Y = 0;// arm.direction().z;
		pointToSend.Position.CartesianPosition.Z = arm.direction().y;
		pointToSend.Position.CartesianPosition.ThetaX = 0;// -arm.direction().x / 3;
		pointToSend.Position.CartesianPosition.ThetaY = 0;
		pointToSend.Position.CartesianPosition.ThetaZ = 0; // arm.direction().y / 3;;

		//if hand is closed, close the fingers likewise when hand is open
		if (hand.grabStrength()  < .1 || hand.grabStrength() == 0 ) {
			pointToSend.Position.Fingers.Finger1 = -5000;
			pointToSend.Position.Fingers.Finger2 = -5000;
			pointToSend.Position.Fingers.Finger3 = -5000;
		}
		else {
			pointToSend.Position.Fingers.Finger1 = 5000;
			pointToSend.Position.Fingers.Finger2 = 5000;
			pointToSend.Position.Fingers.Finger3 = 5000;
		}

		for (int i = 0; i < 100; i++) {
			//We move the robot to the goal every 100 ms
			MySendBasicTrajectory(pointToSend);
			Sleep(1);
		}
		/*cout << "*********************************" << endl;
		cout << "X   command : " << currentCommand.Coordinates.X << " m" << "     Position : " << currentPosition.Coordinates.X << " m" << endl;
		cout << "Y   command : " << currentCommand.Coordinates.Y << " m" << "     Position : " << currentPosition.Coordinates.Y << " m" << endl;
		cout << "Z   command : " << currentCommand.Coordinates.Z << " m" << "     Position : " << currentPosition.Coordinates.Z << " m" << endl;
		cout << "Theta X   command : " << currentCommand.Coordinates.ThetaX << " Rad" << "     Position : " << currentPosition.Coordinates.ThetaX << " Rad" << endl;
		cout << "Theta Y   command : " << currentCommand.Coordinates.ThetaY << " Rad" << "     Position : " << currentPosition.Coordinates.ThetaY << " Rad" << endl;
		cout << "Theta Z   command : " << currentCommand.Coordinates.ThetaZ << " Rad" << "     Position : " << currentPosition.Coordinates.ThetaZ << " Rad" << endl << endl;
		cout << "*********************************" << endl << endl << endl;*/
	}
}

void SampleListener::onFocusGained(const Leap::Controller& controller) {
	cout << "Focus Gained" << endl;
}

void SampleListener::onFocusLost(const Leap::Controller& controller) {
	cout << "Focus Lost" << endl;
}

void SampleListener::onDeviceChange(const Leap::Controller& controller) {
	cout << "Device Changed" << endl;
	const Leap::DeviceList devices = controller.devices();

	for (int i = 0; i < devices.count(); ++i) {
		cout << "id: " << devices[i].toString() << endl;
		cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << endl;
	}
}

void SampleListener::onServiceConnect(const Leap::Controller& controller) {
	cout << "Service Connected" << endl;
}

void SampleListener::onServiceDisconnect(const Leap::Controller& controller) {
	cout << "Service Disconnected" << endl;
}

int main(int argc, char** argv) {

	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//Initialise the function pointer from the API
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
	MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");

	// Create a sample listener and controller
	SampleListener listener;
	Leap::Controller controller;

	// Have the sample listener receive events from the controller
	controller.addListener(listener);

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) || (MyGetCartesianPosition == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL))

	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		int result = (*MyInitAPI)();

		cout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];

		int devicesCount = MyGetDevices(list, result);

		for (int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			cout << "Initializing the fingers" << endl;
			MyInitFingers();
			pointToSend.InitStruct();
			
			if (argc > 1 && strcmp(argv[1], "--bg") == 0)
				controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

			// Keep this process running until Enter is pressed
			cout << "Press Enter to quit..." << endl;
			cin.get();
			FreeLibrary(commandLayer_handle);

			// Remove the sample listener when done
			controller.removeListener(listener);
		}

		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();
	}
	return 0;
}
