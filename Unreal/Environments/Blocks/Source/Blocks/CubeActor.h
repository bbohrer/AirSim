#pragma once
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "Plan.hpp"
#include "Geom.hpp"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include <exception>
#include <set>
#include "CubeActor.generated.h"


UCLASS(config = Game)
class ACubeActor : public AActor
{
	GENERATED_BODY()

public:
	UPROPERTY(VisibleAnywhere, Category = ProcMesh)
	UMaterial* Material;
	ACubeActor();
	void PostLoad();
	void PostDuplicate(EDuplicateMode::Type ed);
	void PostActorCreated();
	UProceduralMeshComponent* mesh;
	void CreateCube();
	int radiusCm;
	int xCm;
	int yCm;
	int zCm;

};