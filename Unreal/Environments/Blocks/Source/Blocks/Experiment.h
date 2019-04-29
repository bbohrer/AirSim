#pragma once

//the following ifndef -> endif block was copied from AirSim so that rpc 
//builds successfully inside of UE4, don't ask me what it does. -Chris
//
//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first
#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/windowsapiscommonpost.hpp"
STRICT_MODE_ON
#endif
#endif


#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Geom.hpp"
#include "Monitor.hpp"
#include "Plan.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
//#include "CarPawnApi.h"

#include "Experiment.generated.h"

enum ControlMode {
	KEYBOARD = 0,
	PLAN = 1,
	SPIN = 2,
	NUM_MODES
};

enum FeedbackMode {
	BANGBANG = 0,
	PDNORMAL,
	PDNARROW,
	PDWIDE,
	PDOVERDRIVE,
	PDFAST,
	PDSLOW,
	HUMANBOX
};

enum Level {
	LRECT = 0,
	LGRIDS,
	LCLOVER
};

UCLASS()
class BLOCKS_API AExperiment : public AActor
{
	GENERATED_BODY()
private:
	pt2 lastDir;
	// The file that the CSV log will be written to. See printConsts()
	Monitor m;
	Plan plan_;
	bool cruise, cruiseDir, cruiseDirSet, close, slowClose;
	double lastTime_;
	ControlMode mode_;
	FeedbackMode fb_;
	Level level_;
	int curNode_;
	std::unique_ptr<msr::airlib::CarApiBase> vehicle_api_;
	msr::airlib::CarApiBase::CarControls* keyboard_controls_;
	msr::airlib::CarApiBase::CarControls current_controls_;
	msr::airlib::CarApiBase::CarControls ai_controls;
	NodeDatum curND_, realCurND_;

	void loadRect();
	void loadGrids();
	void loadClover();
	void gridTo(double a, double b, double c, double d, double e, double f, double g);
	void aTo(double r, double cornX, double cornY, double fromX, double fromY, double toX, double toY, double f, double g, bool ccw);
	void parcTo(double rad, pt2 to, pt2 center, pt2 from, bool cw);
	bool atEnd(pt2 p, pt2 d, double v);
	int ctrlTicks;

public:
	AExperiment();
	// Sets default values for this actor's properties
	AExperiment(msr::airlib::CarApiBase::CarControls*  keyboard_controls);
protected:
	virtual void BeginPlay() override;
	virtual void LoadLevel();
public:
	virtual void Tick(float DeltaTime) override;
//	virtual void StartServer();
};
