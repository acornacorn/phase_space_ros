#pragma once

#include <stdint.h>

#define ATLAS_SIM_PORT			5062			// port number for IP comm
#define ATLAS_SIM_PORT_RETURN	5063			// port number for returning packet
#define ATLAS_SIM_MSG_TAG		0x234144		// Old tag ID (NOT USED)
#define ATLAS_SIM_MSG_TAG2		0x234145		// Identifies a valid message

#pragma pack(1)

enum MsgType {
    MSG_Camera = 0,   // Packet describes the camera arm pose
    MSG_Arm = 1,      // Packet describes an arm pose (armId identifies which)
    MSG_Ghost = 2,    // Packet describes a "ghost hand" pose (armId identifies which)
};

// Camera is an arm with the camera on it which participates in collision detection and dynamics.
// Arm is an arm with a gripper which participates in collision detection and dynamics.
// Ghost is a gripper only.  It does not participate in collision detection or dynamics.

typedef struct {
	// set to ATLAS_SIM_MSG_TAG2
    int32_t tag;

	// incremental count to check repeated msg
    int32_t pckId;

	// // one of the MsgType enum values
    int32_t msgType;

	// 0, 1, 2, 3 - identifies which Arm or Ghost hand this packet describes. 
	// Always 0 for Camera.
    int32_t armId;

	// Position and orientation of the Arm's or Ghost hand's gripper.
	// For camera only position and up vector matter.
    double mat[4][4];

	// Position of trocar for arms and camera.  Not used for Ghost.
    double trocar[3];

	// Grip value.  0=closed, 100=fully open
    double grip; 

	// Time the packet was sent (seconds since some fixed point in time)
    double tsSent;

	// Color for drawing the Arm's or Ghost hand's gripper.
	// Ignored for camera.
	// Format is hex 0xaabbggrr
	//  aa = alpha
	//  rr = red
	//  gg = green
	//  bb = blue
	// If alpha is 0, do not draw this arm or Ghost hand at all, and (for arms)
	// remove the arm entirely from the world so it does not participate in
	// collision detection or dynamics.
    int32_t color;
} AtlasSimMsg;


#pragma pack()




