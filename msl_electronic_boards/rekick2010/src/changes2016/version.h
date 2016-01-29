
// WARNING: THIS VERSION IS UNTESTED.
// New features:
// * Version change 1.2. This Firmware version is not accepted by
//   the rekick driver written of the framework. Add the following line to
//   trunk/src/Driver/ReKick.cs near line 123:
//         this.firmwareVersions.Add("RK1.2");
//   REMOVE any line which allows Version 1.2
// * new define directive "DEACTIVATE_SERVO" declared. Uncomment this to
//   deactivate the servo and make sure the hardware is fixed in a safe
//   position. (Untested, but should work)
// * new define directive USE_SOLENOID_INTERLOCK declared. Uncomment this
//   to add the new hardware locking mechanism. Completly untested.
//
// Some notes on the meaning of the signal ROTOR_INTERLOCK:
// * A LOW signal means LOCK
// * A HIGH signal means UNLOCK.
//
// Test todo: Check both directives in each combination on a working target
// without attached solenoid.
//
// When everything is working you can remove this comment and add the new
// firmware version to the repository
#warning THIS FIRMWARE VERSION IS UNTESTED. CHECK version.h FOR MORE INFORMATION

#ifndef VERSION_H
#define VERSION_H

// version number
#define MAJOR	1
#define MINOR	1

#endif
