// *************************************************************************************************
//										C o o r d T r a n s l a t e . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Coordinate Translation function declarations
//
// *************************************************************************************************

void ConvertSPAtoLocalOrientation(SmartTrakOrientation *ptrOrientation);
void ConvertSPAMoveDegreesToTicks(PTR_SPA_MOVE_INFO ptrSPAMove, PTR_ORIENTATION const ptrCurrentLocalOrientation);

float ConvertMSITicksToDegrees(INT32 lTicks, enum tagAxis eAxis);		// does not work?
INT32 ConvertDegreesToMSITicks(float fDegrees, enum tagAxis eAxis);


// end of CoordTranslate.h

