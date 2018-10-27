#include "safety/CubeActor.h"
#include "ConstructorHelpers.h"
#include "Engine.h"
//static int Mmagg = 100 * ;
//static int xoff = 0;
//static int yoff = 0;
ACubeActor::ACubeActor() {
	//char buf[256];
	//snprintf(buf, 256, "%d", Mmagg);
	radiusCm = xCm = yCm = zCm = 0;
	//xoff += MAG;
	//yoff += MAG;
	//Mmagg *= 2;
	mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("CubeActor"));
	RootComponent = mesh;
	//ConstructorHelpers::FObjectFinder<UMaterial> MaterialOb(TEXT("Material'/Game/VertexMat.VertexMat'"));
	//Material =  GEngine->WireframeMaterial;
		//MaterialOb.Object;
	//mesh->RegisterComponent();
	//mesh->bUseAsyncCooking = true;
}

void ACubeActor::PostActorCreated() {
	//Super::PostActorCreated(); CreateCube();
}


void ACubeActor::PostDuplicate(EDuplicateMode::Type ed) {
	//Super::PostDuplicate(ed); CreateCube();
}


void ACubeActor::PostLoad() { 
	//Super::PostLoad(); CreateCube(); 
}

void ACubeActor::CreateCube() {
	FVector lll(xCm-radiusCm, yCm-radiusCm, zCm-radiusCm), llr(xCm-radiusCm, yCm-radiusCm, zCm+radiusCm), 
		    lrl(xCm-radiusCm, yCm + radiusCm, zCm-radiusCm), lrr(xCm-radiusCm, yCm + radiusCm, zCm+radiusCm),
	        rll(xCm + radiusCm, yCm-radiusCm, zCm-radiusCm), rlr(xCm + radiusCm, yCm-radiusCm, zCm+radiusCm), 
		    rrl(xCm+radiusCm, yCm+radiusCm, zCm-radiusCm), rrr(xCm+radiusCm, yCm+radiusCm, zCm+radiusCm);
	TArray<FVector> vertices;
	vertices.Add(lll); vertices.Add(llr); vertices.Add(lrl); vertices.Add(lrr);
	vertices.Add(rll); vertices.Add(rlr); vertices.Add(rrl); vertices.Add(rrr);
	TArray<int32> Triangles;
	int tris[] =
	{
		4,0,2,  6,4,2, //bot
		6,4,5,  7,6,5, //right
		1,4,0,  5,4,1, //front
		2,1,0,  3,1,2, //left
		7,5,1,  7,1,3, //top
		6,3,2,  3,6,7  //back
	};
	for (int i = 0; i < (sizeof(tris) / sizeof(tris[0])); i++) {
		Triangles.Add(tris[i]);
	}
	/*Triangles.Add(2); Triangles.Add(0); Triangles.Add(4); //bot
	Triangles.Add(2); Triangles.Add(4); Triangles.Add(6); //bot
	Triangles.Add(5); Triangles.Add(4); Triangles.Add(6); //right
	Triangles.Add(5); Triangles.Add(6); Triangles.Add(7); //right
	Triangles.Add(0); Triangles.Add(4); Triangles.Add(1); //front
	Triangles.Add(1); Triangles.Add(4); Triangles.Add(5); //front
	Triangles.Add(0); Triangles.Add(1); Triangles.Add(2); //left
	Triangles.Add(2); Triangles.Add(1); Triangles.Add(3); //left
	Triangles.Add(1); Triangles.Add(5); Triangles.Add(7); //top
	Triangles.Add(3); Triangles.Add(1); Triangles.Add(7); //top
	Triangles.Add(2); Triangles.Add(3); Triangles.Add(6); //back
	Triangles.Add(7); Triangles.Add(6); Triangles.Add(3); //back*/

	TArray<FVector> normals;
	normals.Add(FVector(0, 0, -1)); normals.Add(FVector(0, 0, -1)); normals.Add(FVector(0, 0, -1)); //bot
	normals.Add(FVector(0, 0, -1)); normals.Add(FVector(0, 0, -1)); normals.Add(FVector(0, 0, -1)); //bot
	normals.Add(FVector(1, 0, 0)); normals.Add(FVector(1, 0, 0)); normals.Add(FVector(1, 0, 0));//right
	normals.Add(FVector(1, 0, 0)); normals.Add(FVector(1, 0, 0)); normals.Add(FVector(1, 0, 0));//right
	normals.Add(FVector(0, -1, 0)); normals.Add(FVector(0, -1, 0)); normals.Add(FVector(0, -1, 0));//front
	normals.Add(FVector(0, -1, 0)); normals.Add(FVector(0, -1, 0)); normals.Add(FVector(0, -1, 0));//front
	normals.Add(FVector(-1, 0, 0)); normals.Add(FVector(-1, 0, 0)); normals.Add(FVector(-1, 0, 0));//left
	normals.Add(FVector(-1, 0, 0)); normals.Add(FVector(-1, 0, 0)); normals.Add(FVector(-1, 0, 0));//left
	normals.Add(FVector(0, 0, 1)); normals.Add(FVector(0, 0, 1)); normals.Add(FVector(0, 0, 1));//top
	normals.Add(FVector(0, 0, 1)); normals.Add(FVector(0, 0, 1)); normals.Add(FVector(0, 0, 1));//top
	normals.Add(FVector(0, 1, 0)); normals.Add(FVector(0, 1, 0)); normals.Add(FVector(0, 1, 0));//back
	normals.Add(FVector(0, 1, 0)); normals.Add(FVector(0, 1, 0)); normals.Add(FVector(0, 1, 0));//back

	TArray<FVector2D> UV0;
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//bot
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//bot
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//right
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//right
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//front
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//front
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//left
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//left
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//top
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//top
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//back
	UV0.Add(FVector2D(0, 0)); UV0.Add(FVector2D(10, 0)); UV0.Add(FVector2D(0, 10));//back


	TArray<FProcMeshTangent> tangents;
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//bot
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//bot
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//right
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//right
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//front
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//front
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//left
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//left
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//top
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//top
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//back
	tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0)); tangents.Add(FProcMeshTangent(0, 1, 0));//back

	TArray<FLinearColor> vertexColors;
	FLinearColor gray = FLinearColor(1.0, 0.0, 0.0, 0.75);
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//bot
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//bot
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//right
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//right
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//front
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//front
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//left
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//left
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//top
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//top
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//back
	vertexColors.Add(gray); vertexColors.Add(gray); vertexColors.Add(gray);//back

	// tangents = TArray<FProcMeshTangent>()
	//UV0 = TArray<FVector2D>()
	//normals = TArray<FVector>()
	mesh->CreateMeshSection_LinearColor(0, vertices, Triangles, normals, UV0, vertexColors, tangents, false);
	//mesh->SetMaterial(0, Material);

	// Enable collision data
	//mesh->ContainsPhysicsTriMeshData(true);
	bool isVisible = false;
	mesh->SetVisibility(isVisible, true);
}