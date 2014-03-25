// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"

#include <moai-sim/MOAISkyways.h>
#include <moai-sim/MOAIVertexBuffer.h>
#include <moai-sim/MOAIVertexFormat.h>

#include <clipper/clipper.hpp>

#include <libtess2/Include/tesselator.h>

#ifdef MOAI_USE_SHINY
	#include "ShinyLua.h"
#endif

// TODO: these are getting reintroduced somewhere; find them and kill them

#ifdef DeleteFile
	#undef DeleteFile
#endif

#ifdef RemoveDirectory
	#undef RemoveDirectory
#endif

using namespace ClipperLib;

//================================================================//
// STRUCT
//================================================================//

struct FVertex {
public:
	USVec2D mV;
	USVec2D mN;
	u32 color;
	FVertex(USVec2D v): mV(v) {};
	FVertex(USVec2D v, u32 c): mV(v), color(c) {};
};

typedef std::vector< FVertex > FPolygon;
typedef std::vector< FPolygon > FPolygons;
typedef std::vector< u32 > Colors;

//================================================================//
// LUA TABLE TRANSLATE
//================================================================//

float static popTableFloat(MOAILuaState state, lua_State* L, int tableIndex)
{
	
	lua_next ( L, tableIndex );
	float val = state.GetValue < float >( -1, 0.0f );
	lua_pop ( L, 1 );
	return val;
}

void static readPolyLinesFromLua(FPolygons &polyLines, MOAILuaState state, lua_State* L, int tableIndex)
{
	
	int length = luaL_getn(L, tableIndex);
	
	float x1, y1, x2, y2, r, g, b, a;

	u32 lineCount = 0;
	lua_pushnil ( L );
	while (lineCount < length / 8) {
		x1	= popTableFloat(state, L, tableIndex);
		y1	= popTableFloat(state, L, tableIndex);
		x2	= popTableFloat(state, L, tableIndex);
		y2	= popTableFloat(state, L, tableIndex);
		r	= popTableFloat(state, L, tableIndex);
		g	= popTableFloat(state, L, tableIndex);
		b	= popTableFloat(state, L, tableIndex);
		a	= popTableFloat(state, L, tableIndex);
		
		u32 color = ZLColor::PackRGBA(r, g, b, a);
		ZLColorVec cv = ZLColor::Set(color);

		FPolygon polyLine;
		polyLine.push_back(FVertex(USVec2D(x1, y1), color));
		polyLine.push_back(FVertex(USVec2D(x2, y2), color));
		polyLines.push_back(polyLine);

		++lineCount;
	}
}

void static readIntersectionPointsFromLua(FPolygon &intersectionPoints, MOAILuaState state, lua_State* L, int tableIndex)
{
	int length = luaL_getn(L, tableIndex);
	
	float x, y;
	intersectionPoints.clear();

	lua_pushnil ( L );
	int compCount = 0;
	while ( lua_next ( L, tableIndex ) != 0 ) {
		if ( compCount % 2 == 0 ) {
			x = state.GetValue < float >( -1, 0.0f );
		} else {
			y = state.GetValue < float >( -1, 0.0f );
			intersectionPoints.push_back(FVertex(USVec2D(x, y)));
		}
		++compCount;
		lua_pop ( L, 1 );
	}
}

void static readPerpLegIndexesFromLua(vector<int> &physics_perpLegIndexes, MOAILuaState state, lua_State* L, int tableIndex)
{
	physics_perpLegIndexes.clear();
	
	lua_pushnil ( L );
	while ( lua_next ( L, tableIndex ) != 0 ) {
		cc8* stringIndex = state.GetValue < cc8* >( -1, "" );
		int index = atoi(stringIndex);
		physics_perpLegIndexes.push_back(index);
		lua_pop ( L, 1 );
	}
}

void static setTableFloat(lua_State* L, int tableIndex, int index, float value)
{
	lua_pushnumber(L, value); // [bt, i, tt, x]
	lua_rawseti (L, tableIndex, index); // [bt, i, tt]
}

void static pushPhysicsSegmentsToLua(MOAILuaState state, lua_State* L, const FPolygons &polygons)
{
	int count = polygons.size();
	lua_createtable(L, count, 0); // [bt]
	int i = 0;
	for(FPolygons::const_iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon, ++i) {
		int compCount = itPolygon->size();
		lua_pushinteger(L, i + 1); // [bt, i]
		int colorOffset = 4;
		lua_createtable(L, compCount * 2 + colorOffset, 0); // [bt, i, tt]
		{
			FVertex vertex = itPolygon->at(0);
			ZLColorVec color = ZLColor::Set(vertex.color);
			setTableFloat(L, -2, 1, color.mR);
			setTableFloat(L, -2, 2, color.mG);
			setTableFloat(L, -2, 3, color.mB);
			setTableFloat(L, -2, 4, color.mA);
		}
		for (int j = 0; j < compCount; j++) {
			setTableFloat(L, -2, j * 2 + 1 + colorOffset, itPolygon->at(j).mV.mX);
			setTableFloat(L, -2, j * 2 + 2 + colorOffset, itPolygon->at(j).mV.mY);
		}
		lua_settable(L, -3); // [bt[i=tt]
	}
}

int static countLinesInPolygons(FPolygons polygons) {
	int count = 0;
	for(FPolygons::const_iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon) {
		count += itPolygon->size();
	}
	return count;
}

int static roundToInt(float d)
{
  return floor(d + 0.5);
}

void static pushPerpCollisionPositionsToLua(MOAILuaState state, lua_State* L, const FPolygons &physics_perpIntersectionPolygons, const float gridSize)
{
	// horrible data format (efficient)
	// legindex, bottomy, topy, legindex, bottomy, topy, ...
	// "5", 4, 4.3, "11", 5.6, 5.75, ...
	// use strings for lua proper precision

	float top, bottom, left, right;
	char stringGridIndex[15];

	lua_createtable(L, physics_perpIntersectionPolygons.size() * 3, 0); // [t]
	int i = 0;
	for(FPolygons::const_iterator itPolygon = physics_perpIntersectionPolygons.begin(); itPolygon != physics_perpIntersectionPolygons.end(); ++itPolygon, ++i) {
		top = -1000; bottom = 1000; left = 1000; right = -1000;
		for(FPolygon::const_iterator itVertex = itPolygon->begin(); itVertex != itPolygon->end(); ++itVertex) {
			top		= max(top, itVertex->mV.mY);
			bottom	= min(bottom, itVertex->mV.mY);
			left	= min(left, itVertex->mV.mX);
			right	= max(right, itVertex->mV.mX);
		}
		float indexWorldLoc = (left + right)/2;
		int gridIndex = roundToInt(indexWorldLoc/gridSize);		
		sprintf(stringGridIndex, "%d", gridIndex);

		lua_pushstring(L, stringGridIndex); // [t, s]
		lua_rawseti (L, -2, i * 3 + 1); // [t]
		lua_pushnumber(L, bottom); // [t, n]
		lua_rawseti (L, -2, i * 3 + 1 + 1); // [t]
		lua_pushnumber(L, top); // [t, n]
		lua_rawseti (L, -2, i * 3 + 2 + 1); // [t]
	}
}

//================================================================//
// INTERSECTION GEOMS
//================================================================//

void static createIntersectionGeometries(const FPolygon &intersectionPoints, FPolygons &cutIntersectionGeometries, float delta,
	bool physics_generateSegments, FPolygons &physics_unionIntersectionGeometries, FPolygons &physics_cutIntersectionGeometries)
{	
	cutIntersectionGeometries.clear();
	for(FPolygon::const_iterator itVertex = intersectionPoints.begin(); itVertex != intersectionPoints.end(); ++itVertex) {
		USVec2D p = itVertex->mV;
		FPolygon geom;
		float expandX = 1.001, expandY = 1.2;
		geom.push_back(USVec2D(p.mX - delta * expandX, p.mY + delta * expandY));
		geom.push_back(USVec2D(p.mX - delta * expandX, p.mY - delta * expandY));
		geom.push_back(USVec2D(p.mX + delta * expandX, p.mY - delta * expandY));
		geom.push_back(USVec2D(p.mX + delta * expandX, p.mY + delta * expandY));
		cutIntersectionGeometries.push_back(geom);
	}
	
	if (physics_generateSegments) {
		physics_unionIntersectionGeometries.clear();
		physics_cutIntersectionGeometries.clear();
		for(FPolygon::const_iterator itVertex = intersectionPoints.begin(); itVertex != intersectionPoints.end(); ++itVertex) {
			USVec2D p = itVertex->mV;
			FPolygon unionGeom;
			unionGeom.push_back(USVec2D(p.mX - delta, p.mY + delta));
			unionGeom.push_back(USVec2D(p.mX - delta, p.mY - delta));
			unionGeom.push_back(USVec2D(p.mX + delta, p.mY - delta));
			unionGeom.push_back(USVec2D(p.mX + delta, p.mY + delta));
			physics_unionIntersectionGeometries.push_back(unionGeom);

			FPolygon cutGeom;
			float expandX = 1.2, expandY = 1.2;
			cutGeom.push_back(USVec2D(p.mX - delta * expandX, p.mY + delta * expandY));
			cutGeom.push_back(USVec2D(p.mX - delta * expandX, p.mY - delta * expandY));
			cutGeom.push_back(USVec2D(p.mX + delta * expandX, p.mY - delta * expandY));
			cutGeom.push_back(USVec2D(p.mX + delta * expandX, p.mY + delta * expandY));
			physics_cutIntersectionGeometries.push_back(cutGeom);
		}
	}
}

//================================================================//
// OFFSETTING
//================================================================//

void static	floatToIntScale(const FPolygons &polys, Polygons &scaledPolys, float scale) {
	
	scaledPolys.clear();
	for(FPolygons::const_iterator itPolygon = polys.begin(); itPolygon != polys.end(); ++itPolygon) {
		ClipperLib::Polygon scaledPoly;
		for (int j = 0; j < itPolygon->size(); ++j) {
			USVec2D p = (*itPolygon)[j].mV;
			scaledPoly.push_back(IntPoint(p.mX * scale, p.mY * scale));
		}
		scaledPolys.push_back(scaledPoly);
	}
}

void static	intToFloatScale(const Polygons &scaledPolys, FPolygons &polys, float scale) {
	
	polys.clear();
	for(Polygons::const_iterator itScaledPoly = scaledPolys.begin(); itScaledPoly != scaledPolys.end(); ++itScaledPoly) {
		FPolygon poly;
		for (int j = 0; j < itScaledPoly->size(); ++j) {
			IntPoint p = (*itScaledPoly)[j];
			poly.push_back(FVertex(USVec2D(p.X * scale, p.Y * scale)));
		}
		polys.push_back(poly);
	}
}

static Polygons createPerpendicularIntersectionTestPolygons(const vector<int> &physics_perpLegIndexes, const float delta, const float scale, const float gridSize)
{
	const float scaledGridSize = gridSize * scale;
	const float scaledDelta = delta * scale;

	// try to be higher/lower then a level would ever be
	const int top = 1000 * scale;
	const int bottom = -1000 * scale;

	Polygons scaledPolygons;
	for(vector<int>::const_iterator itIndex = physics_perpLegIndexes.begin(); itIndex != physics_perpLegIndexes.end(); ++itIndex) {
		int index = *itIndex;
		float worldLocForIndex = scaledGridSize * index;

		int left	= worldLocForIndex - scaledDelta;
		int right	= worldLocForIndex + scaledDelta;

		ClipperLib::Polygon polygon;
		polygon.push_back(IntPoint( right, bottom ));
		polygon.push_back(IntPoint( right, top	  ));
		polygon.push_back(IntPoint( left,  top	  ));
		polygon.push_back(IntPoint( left,  bottom ));
		scaledPolygons.push_back(polygon);
	}
	return scaledPolygons;
}

static void offsetPolyLinesToPolygons(const FPolygons &polyLines, const float offsetAmount, const float geom_roundLimit, FPolygons &geom_cutIntersectionGeometries, FPolygons &geom_cutPolygons, vector<bool> &geom_cutPolygonOrientations, 
	bool physics_generateSegments, const float physics_roundLimit, const float gridSize, const vector<int> &physics_perpLegIndexes, const FPolygons &physics_unionIntersectionGeometries, const FPolygons &physics_cutIntersectionGeometries, FPolygons &physics_unionPolygons, vector<bool> &physics_unionPolygonOrientations, FPolygons &physics_perpIntersectionPolygons) {
	
	const float scale = 100000.0f; // scale for integers used by clipper

	Polygons scaledPolyLines, geom_scaledCutIntersectionGeometries, geom_scaledPolygons, geom_scaledCutPolygons;
	floatToIntScale(polyLines, scaledPolyLines, scale);
	
 	OffsetPolyLines(scaledPolyLines, geom_scaledPolygons, offsetAmount * scale, jtRound, etRound, geom_roundLimit * scale);
	
	floatToIntScale(geom_cutIntersectionGeometries, geom_scaledCutIntersectionGeometries, scale);

    Clipper clpr;
    clpr.AddPolygons(geom_scaledPolygons, ptSubject);
	clpr.AddPolygons(geom_scaledCutIntersectionGeometries, ptClip);
    clpr.Execute(ctDifference, geom_scaledCutPolygons, pftPositive, pftPositive);

	intToFloatScale(geom_scaledCutPolygons, geom_cutPolygons, 1.0f/scale);

	geom_cutPolygonOrientations.clear();
	for(Polygons::const_iterator itScaledCutPoly = geom_scaledCutPolygons.begin(); itScaledCutPoly != geom_scaledCutPolygons.end(); ++itScaledCutPoly) {
		geom_cutPolygonOrientations.push_back(Orientation(*itScaledCutPoly));
	}
	
	if (physics_generateSegments) {
		Polygons physics_scaledPolygons, physics_scaledUnionIntersectionGeometries, physics_scaledCutIntersectionGeometries, physics_scaledUnionPolygons, physics_scaledCutPolygons, physics_scaledPerpIntersectionPolygons;

 		OffsetPolyLines(scaledPolyLines, physics_scaledPolygons, offsetAmount * scale, jtRound, etRound, physics_roundLimit * scale);

		floatToIntScale(physics_unionIntersectionGeometries, physics_scaledUnionIntersectionGeometries, scale);
		floatToIntScale(physics_cutIntersectionGeometries, physics_scaledCutIntersectionGeometries, scale);

		clpr.Clear();
		clpr.AddPolygons(physics_scaledPolygons, ptSubject);
		clpr.AddPolygons(physics_scaledUnionIntersectionGeometries, ptClip);
		clpr.Execute(ctUnion, physics_scaledUnionPolygons, pftPositive, pftPositive);
	
		clpr.Clear();
		clpr.AddPolygons(physics_scaledPolygons, ptSubject);
		clpr.AddPolygons(physics_scaledCutIntersectionGeometries, ptClip);
		clpr.Execute(ctDifference, physics_scaledCutPolygons, pftPositive, pftPositive);

		clpr.Clear();
		clpr.AddPolygons(createPerpendicularIntersectionTestPolygons(physics_perpLegIndexes, offsetAmount, scale, gridSize), ptSubject);
		clpr.AddPolygons(physics_scaledCutPolygons, ptClip);
		clpr.Execute(ctIntersection, physics_scaledPerpIntersectionPolygons, pftPositive, pftPositive);
		
		intToFloatScale(physics_scaledUnionPolygons, physics_unionPolygons, 1.0f/scale);
		intToFloatScale(physics_scaledPerpIntersectionPolygons, physics_perpIntersectionPolygons, 1.0f/scale);

		physics_unionPolygonOrientations.clear();
		for(Polygons::const_iterator itPoly = physics_scaledUnionPolygons.begin(); itPoly != physics_scaledUnionPolygons.end(); ++itPoly) {
			physics_unionPolygonOrientations.push_back(Orientation(*itPoly));
		}
	}
}

//================================================================//
// NORMALS
//================================================================//

static void computeNormalsForPolygon(FPolygon &polygon, bool orientation) {
	
	int size = polygon.size();
	for (int i = 0; i < size; ++i) {
		FVertex& vertex = polygon[i];
		
		USVec2D vRight = polygon[(i + size - 1)%size].mV;
		USVec2D vMiddle = vertex.mV;
		USVec2D vLeft = polygon[(i + 1)%size].mV;

		USVec2D dir1 = vMiddle - vLeft; dir1.Norm();
		USVec2D dir2 = vRight - vMiddle; dir2.Norm();

		bool isIntersectionCorner = false;
		if (orientation == true) {
			USVec2D dir1rotated = dir1;
			dir1rotated.Rotate90Anticlockwise(); // really clockwise ;)
			
			if (ZLFloat::IsClose(dir1rotated.mX, dir2.mX, EPSILON) && ZLFloat::IsClose(dir1rotated.mY, dir2.mY, EPSILON)) {
				// vertex must be a cut intersection corner
				isIntersectionCorner = true;
				if (dir2.mX > 0 || dir1.mX > 0) {
					// up
					vertex.mN = USVec2D(0.0f, 1.0f);
				} else if (dir2.mX < 0 || dir1.mX < 0) {
					// down
					vertex.mN = USVec2D(0.0f, -1.0f);
				}
			}
		}

		if (!isIntersectionCorner) {
			dir1.Rotate90Clockwise(); // really anticlockwise ;)
			dir2.Rotate90Clockwise(); // really anticlockwise ;)

			vertex.mN = dir1 + dir2;
			vertex.mN.Norm();
		}
	}
}

static void computeNormalsForPolygons(FPolygons &polygons, const vector<bool> &polygonOrientations) {
	int i = 0;
	for(FPolygons::iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon, ++i) {
		computeNormalsForPolygon(*itPolygon, polygonOrientations[i]);
	}
}

//================================================================//
// COLORIZE
//================================================================//

// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
/*
int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((verty[i]>testy) != (verty[j]>testy)) &&
				(testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
			c = !c;
	}
	return c;
}
*/

// Translated from above
static bool pointInPolygon(const FPolygon &polygon, const USVec2D &point) {
	int nvert = polygon.size();
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++) {
		if ( ((polygon[i].mV.mY>point.mY) != (polygon[j].mV.mY>point.mY)) &&
				(point.mX < (polygon[j].mV.mX-polygon[i].mV.mX) * (point.mY-polygon[i].mV.mY) / (polygon[j].mV.mY-polygon[i].mV.mY) + polygon[i].mV.mX) )
			c = !c;
	}
	return c;
}

static void setPolygonColors(FPolygon &polygon, const u32 color) {
	for(FPolygon::iterator itVertex = polygon.begin(); itVertex != polygon.end(); ++itVertex) {
		itVertex->color = color;
	}
}

static u32 computeColorForPolygon(FPolygon &polygon, const std::vector< FVertex > &pointsWithColors) {
	for(std::vector< FVertex >::const_iterator itPoint = pointsWithColors.begin(); itPoint != pointsWithColors.end(); ++itPoint) {
		if (pointInPolygon(polygon, itPoint->mV)) {
			return itPoint->color;
		}
	}
	return NULL;
}

static void flattenPolyLinesToPoints(const FPolygons &polyLines, std::vector< FVertex > &points) {
	for(FPolygons::const_iterator itLine = polyLines.begin(); itLine != polyLines.end(); ++itLine) {
		points.push_back(itLine->at(0));
		points.push_back(itLine->at(1));
	}
}

const u32 black = ZLColor::PackRGBA(0.0f, 0.0f, 0.0f, 1.0f);
static void computeColorsForPolygons(FPolygons &polygons, const FPolygons &polyLines, const vector<bool> &polygonOrientations) {
	std::vector< FVertex > pointsWithColors;
	flattenPolyLinesToPoints(polyLines, pointsWithColors);
	u32 lastColor = NULL;
	int i = 0;
	for(FPolygons::iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon, ++i) {
		bool orientation = polygonOrientations[i];
		//MOAIPrint(orientation ? ">" : "<");
		u32 color;
		if (orientation)
			color = computeColorForPolygon(*itPolygon, pointsWithColors);
		else
			color = lastColor;
		if (!color) {
			MOAIPrint("Warning, found no color for polygon, defaulting to black\n");
			color = black;
		}
		setPolygonColors(*itPolygon, color);
		lastColor = color;
	}
}

//================================================================//
// TESSELATION
//================================================================//

void* stdAlloc(void* userData, unsigned int size)
{
	int* allocated = ( int*)userData;
	*allocated += (int)size;
	return malloc(size);
}

void stdFree(void* userData, void* ptr)
{
	free(ptr);
}

void static tesselatePolygons(const FPolygons &polygons, FPolygons &triangles) {
	
	int allocated = 0;
	TESSalloc ma;
	memset(&ma, 0, sizeof(ma));
	ma.memalloc = stdAlloc;
	ma.memfree = stdFree;
	ma.userData = (void*)&allocated;
	ma.extraVertices = 256; // realloc not provided, allow 256 extra vertices.

	TESStesselator* tess = tessNewTess(&ma);

	FPolygon vertexIndex;
	
	for(FPolygons::const_iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon) {
		int count = itPolygon->size();
		float *comps = new float[count * 2];
		for (int j = 0; j < count; ++j) {
			FVertex vertex = (*itPolygon)[j];
			comps[j * 2] = vertex.mV.mX;
			comps[j * 2 + 1] = vertex.mV.mY;
			vertexIndex.push_back(vertex);
		}
		tessAddContour(tess, 2, comps, sizeof(float)*2, count);
	}

	tessTesselate(tess, TESS_WINDING_POSITIVE, TESS_POLYGONS, 3, 2, 0);
	
	const float* verts = tessGetVertices(tess);
	const int* vinds = tessGetVertexIndices(tess);
	const int* elems = tessGetElements(tess);
	const int nverts = tessGetVertexCount(tess);
	const int nelems = tessGetElementCount(tess);

	for (int i = 0; i < nelems; ++i)
	{
		const int* p = &elems[i*3];
		FPolygon poly;
		for (int j = 0; j < 3 && p[j] != TESS_UNDEF; ++j) {
			int vertexNumber = p[j];
			int originalVertexIndex = vinds[vertexNumber];
			FVertex vertex = vertexIndex[originalVertexIndex];
			poly.push_back(vertex);
		}	
		triangles.push_back(poly);
	}

	if (tess) tessDeleteTess(tess);
}

//================================================================//
// 3D
//================================================================//

static inline ZLVec3D to3DWithX(const USVec2D &v2, float x) {
	
	return ZLVec3D(x, v2.mX, v2.mY);
}

static inline ZLVec3D to3DWithY(const USVec2D &v2, float y) {
	
	return ZLVec3D(v2.mX, y, v2.mY);
}

void static inline writePointToStream(ZLByteStream* stream, const ZLVec3D &p)
{
	
	stream->Write<float>(p.mX);
	stream->Write<float>(p.mY);
	stream->Write<float>(p.mZ);
}

void static inline writeVertexToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p, const ZLVec3D &n, const u32 color)
{
	
	ZLByteStream* stream = vbo->GetStream();
	writePointToStream(stream, p);
	writePointToStream(stream, n);
	stream->Write < u32 >( color );
	//MOAIPrint("Regular point: %.2f, %.2f, %.2f - normal: %.2f, %.2f, %.2f\n", p.mX, p.mY, p.mZ, n.mX, n.mY, n.mZ);
}

void static inline writeOutlineVertexToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p, const ZLVec3D &n)
{
	
	ZLByteStream* stream = vbo->GetStream();
	writePointToStream(stream, p);
	writePointToStream(stream, n);
	//MOAIPrint("Outline point: %.2f, %.2f, %.2f - normal: %.2f, %.2f, %.2f\n", p.mX, p.mY, p.mZ, n.mX, n.mY, n.mZ);
}

void static writeTrianglesToVBO(MOAIVertexBuffer* mainVbo, MOAIVertexBuffer* outlineVbo, const FPolygons &triangles, const FPolygons &polyLines, u32 hand, float missingDimValue, float normalSign, const bool frontFacing)
{
	// for some reason right hand frontfacing is a special case :)
	bool ascending = hand == MOAISkyways::HAND_RIGHT && frontFacing;
	
	ZLVec3D p3d, n, outline_n;
	for(FPolygons::const_iterator itTriangle = triangles.begin(); itTriangle != triangles.end(); ++itTriangle) {
		int compCount = itTriangle->size();
		for (int j = 0; j < compCount; ++j) {
			FVertex v2d = !ascending ? (*itTriangle)[compCount - j - 1] : (*itTriangle)[j];
			USVec2D p2d = v2d.mV;
			if (hand == MOAISkyways::HAND_LEFT) {
				p3d.mX = missingDimValue;
				p3d.mY = p2d.mX;

				n.mX = normalSign; n.mY = 0.0f; n.mZ = 0.0f;
				outline_n = n + to3DWithX(v2d.mN, 0.0f);
				outline_n.Norm();
			} else {
				p3d.mX = p2d.mX;
				p3d.mY = missingDimValue;

				n.mX = 0.0f; n.mY = normalSign; n.mZ = 0.0f;
				outline_n = n + to3DWithY(v2d.mN, 0.0f);
				outline_n.Norm();
			}
			p3d.mZ = p2d.mY;

			writeVertexToVBO(mainVbo, p3d, n, v2d.color);
			writeOutlineVertexToVBO(outlineVbo, p3d, outline_n);
		}
	}
}

void static writeTriToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p1, const ZLVec3D &n1, const ZLVec3D &p2, const ZLVec3D &n2, const ZLVec3D &p3, const ZLVec3D &n3, const u32 color)
{
	
	writeVertexToVBO(vbo, p1, n1, color);
	writeVertexToVBO(vbo, p2, n2, color);
	writeVertexToVBO(vbo, p3, n3, color);
}

void static writeTriToOutlineVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p1, const ZLVec3D &n1, const ZLVec3D &p2, const ZLVec3D &n2, const ZLVec3D &p3, const ZLVec3D &n3)
{
	
	writeOutlineVertexToVBO(vbo, p1, n1);
	writeOutlineVertexToVBO(vbo, p2, n2);
	writeOutlineVertexToVBO(vbo, p3, n3);
}

void static	writeTopFaceToVBO(MOAIVertexBuffer* mainVbo, MOAIVertexBuffer* outlineVbo, const FVertex &v1, const FVertex &v2, int orientation, u32 hand, float missingDimValue, float delta, const u32 color)
{
	
	ZLVec3D nl, nr, sr, sl; // north/left/south/right
	ZLVec3D nl_n, nr_n, sr_n, sl_n; // normals
	ZLVec3D nl_on, nr_on, sr_on, sl_on; // outline normals
	USVec2D p1 = v1.mV, p2 = v2.mV;

	/*{
		// this tests skips faces that would face away from camera at all times
		USVec2D realLineNormal = p2 - p1;
		realLineNormal.Rotate90Anticlockwise(); // really clockwise..
		realLineNormal.Norm();

		static USVec2D backfacing(0.8, -0.6);
		if (realLineNormal.Dot(backfacing) >= 0.5)
			return;
	}*/

	if (hand == MOAISkyways::HAND_LEFT) {
		nl.mX = missingDimValue - delta;
		nl.mY = p2.mX;
		nl.mZ = p2.mY;
		nl_n = to3DWithX(v2.mN, 0.0f);
		nl_on = nl_n; nl_on.mX = -1.0f; nl_on.Norm();
		
		nr.mX = missingDimValue + delta;
		nr.mY = p2.mX;
		nr.mZ = p2.mY;
		nr_n = to3DWithX(v2.mN, 0.0f);
		nr_on = nr_n; nr_on.mX = 1.0f; nr_on.Norm();
		
		sr.mX = missingDimValue + delta;
		sr.mY = p1.mX;
		sr.mZ = p1.mY;
		sr_n = to3DWithX(v1.mN, 0.0f);
		sr_on = sr_n; sr_on.mX = 1.0f; sr_on.Norm();
		
		sl.mX = missingDimValue - delta;
		sl.mY = p1.mX;
		sl.mZ = p1.mY;
		sl_n = to3DWithX(v1.mN, 0.0f);
		sl_on = sl_n; sl_on.mX = -1.0f; sl_on.Norm();
	} else {
		nl.mX = p2.mX;
		nl.mY = missingDimValue + delta;
		nl.mZ = p2.mY;
		nl_n = to3DWithY(v2.mN, 0.0f);
		nl_on = nl_n; nl_on.mY = +1.0f; nl_on.Norm();
		
		nr.mX = p2.mX;
		nr.mY = missingDimValue - delta;
		nr.mZ = p2.mY;
		nr_n = to3DWithY(v2.mN, 0.0f);
		nr_on = nr_n; nr_on.mY = -1.0f; nr_on.Norm();
		
		sr.mX = p1.mX;
		sr.mY = missingDimValue - delta;
		sr.mZ = p1.mY;
		sr_n = to3DWithY(v1.mN, 0.0f);
		sr_on = sr_n; sr_on.mY = -1.0f; sr_on.Norm();
		
		sl.mX = p1.mX;
		sl.mY = missingDimValue + delta;
		sl.mZ = p1.mY;
		sl_n = to3DWithY(v1.mN, 0.0f);
		sl_on = sl_n; sl_on.mY = 1.0f; sl_on.Norm();
	}

	writeTriToVBO(mainVbo, nl, nl_n, sr, sr_n, sl, sl_n, color);
	writeTriToOutlineVBO(outlineVbo, nl, nl_on, sr, sr_on, sl, sl_on);

	writeTriToVBO(mainVbo, nl, nl_n, nr, nr_n, sr, sr_n, color);
	writeTriToOutlineVBO(outlineVbo, nl, nl_on, nr, nr_on, sr, sr_on);
}

void static	writeTopFacesToVBO(MOAIVertexBuffer* mainVbo, MOAIVertexBuffer* outlineVbo, const FPolygons &polygons, const vector<bool> &polygonOrientations, u32 hand, float missingDimValue, float delta)
{
	int iP = 0;
	for(FPolygons::const_iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon, ++iP) {
		for (int iV = 0; iV < itPolygon->size(); ++iV) {
			FVertex v1 = (*itPolygon)[iV];
			FVertex v2 = (*itPolygon)[(iV + 1) % itPolygon->size()];
			writeTopFaceToVBO(mainVbo, outlineVbo, v1, v2, polygonOrientations[iP], hand, missingDimValue, delta, v1.color);
		}
	}
}

//================================================================//
// LUA INTERFACE
//================================================================//

int MOAISkyways::_createLegGeometry ( lua_State* L ) {
	MOAILuaState state ( L );
	if ( !state.CheckParams(1, "UUTTNNNNBNNT") ) return 0;
	
	MOAIVertexBuffer* mainVbo					= state.GetLuaObject < MOAIVertexBuffer >( 1, true );
	MOAIVertexBuffer* outlineVbo				= state.GetLuaObject < MOAIVertexBuffer >( 2, true );
	const int lineTableIndex					= 3;
	const int intersectionPointsTableIndex		= 4;
	const float delta							= state.GetValue<float>(5, 0.15f);
	const float geom_roundLimit					= state.GetValue<float>(6, 0);
	const u32 hand								= state.GetValue < u32 >( 7, MOAISkyways::HAND_LEFT );
	const float missingDimValue					= state.GetValue < float >( 8, 0.0f );
	const bool physics_generateSegments			= state.GetValue < bool >( 9, false );
	const float physics_roundLimit				= state.GetValue<float>(10, 0);
	const float gridSize						= state.GetValue<float>(11, -1);
	const int physicsPerpLegIndexesTableIndex	= 12;
	
	FPolygons polyLines, geom_cutIntersectionGeometries, geom_cutPolygons;
	FPolygon intersectionPoints;
	
	vector<int> physics_perpLegIndexes;
	FPolygons physics_unionIntersectionGeometries, physics_cutIntersectionGeometries, physics_unionPolygons, physics_perpIntersectionPolygons;

	readPolyLinesFromLua(polyLines, state, L, lineTableIndex);
	readIntersectionPointsFromLua(intersectionPoints, state, L, intersectionPointsTableIndex);
	if (physics_generateSegments) {
		readPerpLegIndexesFromLua(physics_perpLegIndexes, state, L, physicsPerpLegIndexesTableIndex);
	}

	createIntersectionGeometries(intersectionPoints, geom_cutIntersectionGeometries, delta, 
		physics_generateSegments, physics_unionIntersectionGeometries, physics_cutIntersectionGeometries);
	
	vector<bool> geom_cutPolygonOrientations, physics_unionPolygonOrientations;
	offsetPolyLinesToPolygons(polyLines, delta, geom_roundLimit, geom_cutIntersectionGeometries, geom_cutPolygons, geom_cutPolygonOrientations, 
		physics_generateSegments, physics_roundLimit, gridSize, physics_perpLegIndexes, physics_unionIntersectionGeometries, physics_cutIntersectionGeometries, physics_unionPolygons, physics_unionPolygonOrientations, physics_perpIntersectionPolygons);
	
	computeNormalsForPolygons(geom_cutPolygons, geom_cutPolygonOrientations);

	computeColorsForPolygons(geom_cutPolygons, polyLines, geom_cutPolygonOrientations);
	computeColorsForPolygons(physics_unionPolygons, polyLines, physics_unionPolygonOrientations);

	FPolygons triangles;
	tesselatePolygons(geom_cutPolygons, triangles);

	int lineCount = countLinesInPolygons(geom_cutPolygons);
	// 2 sides, 3 vertices per triangle, 2 triangles per line, 3 vertices per triangle
	int vertexCount = (triangles.size() * 2 * 3 + lineCount * 2 * 3) * mainVbo->GetFormat()->GetVertexSize ();
	mainVbo->Reserve(vertexCount); 
	outlineVbo->Reserve(vertexCount);

	writeTrianglesToVBO(mainVbo, outlineVbo, triangles, polyLines, hand, missingDimValue - delta, -1.0f, true);
	writeTrianglesToVBO(mainVbo, outlineVbo, triangles, polyLines, hand, missingDimValue + delta, -1.0f, false);

	writeTopFacesToVBO(mainVbo, outlineVbo, geom_cutPolygons, geom_cutPolygonOrientations, hand, missingDimValue, delta);
	
	// TODO implement bless here instead of in LUA
	
	if (physics_generateSegments) {
		pushPhysicsSegmentsToLua(state, L, physics_unionPolygons);
		pushPerpCollisionPositionsToLua(state, L, physics_perpIntersectionPolygons, gridSize);

		return 2;
	} else {
		return 0;
	}
}

int MOAISkyways::_startProfile ( lua_State* L ) {
#ifdef MOAI_USE_SHINY
	ShinyLua_clear(L);
	ShinyLua_start(L);
#endif
	return 0;
}

int MOAISkyways::_stopAndGetProfileFlat ( lua_State* L ) {
#ifdef MOAI_USE_SHINY
	ShinyLua_update(L);
	ShinyLua_stop(L);
	ShinyLua_flat_string(L);
	return 1;
#else
	return 0;
#endif
}

//----------------------------------------------------------------//
void MOAISkyways::RegisterLuaClass ( MOAILuaState& state ) {

	luaL_Reg regTable [] = {
		{ "createLegGeometry",					_createLegGeometry },
		{ "startProfile",						_startProfile },
		{ "stopAndGetProfileFlat",				_stopAndGetProfileFlat },
		{ NULL, NULL }
	};

	luaL_register( state, 0, regTable );
	
	state.SetField ( -1, "HAND_LEFT",					( u32 )MOAISkyways::HAND_LEFT );
	state.SetField ( -1, "HAND_RIGHT",					( u32 )MOAISkyways::HAND_RIGHT );
}
