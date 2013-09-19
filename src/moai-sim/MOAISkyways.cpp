// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"
#include <moai-sim/MOAISkyways.h>
#include <moai-sim/MOAIVertexBuffer.h>

#include <clipper/clipper.hpp>

#include <libtess2/Include/tesselator.h>

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

typedef std::vector< USVec2D > FPolygon;
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

void static readPolyLinesFromLua(FPolygons &polyLines, /*Colors &colors, */MOAILuaState state, lua_State* L, int tableIndex)
{
	int length = luaL_getn(L, tableIndex);
	
	float x1, y1, x2, y2;//, r, g, b, a;

	u32 lineCount = 0;
	lua_pushnil ( L );
    while ( lua_next ( L, tableIndex ) != 0 ) {
		u32 compCount = 0;
		
		lua_pushnil ( L );
		x1	= popTableFloat(state, L, -2);
		y1	= popTableFloat(state, L, -2);
		x2	= popTableFloat(state, L, -2);
		y2	= popTableFloat(state, L, -2);
		/*r	= popTableFloat(state, L, -2);
		g	= popTableFloat(state, L, -2);
		b	= popTableFloat(state, L, -2);
		a	= popTableFloat(state, L, -2);*/
		
		FPolygon polyLine;
		polyLine.push_back(USVec2D(x1, y1));
		polyLine.push_back(USVec2D(x2, y2));
		polyLines.push_back(polyLine);

		/*u32 color = ZLColor::PackRGBA ( r, g, b, a );
		colors.push_back(color);*/

		++lineCount;
		lua_pop ( L, 1 );
		lua_pop ( L, 1 );
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
			intersectionPoints.push_back(USVec2D(x, y));
		}
		++compCount;
		lua_pop ( L, 1 );
	}
}

void static pushPolygonsToLua(MOAILuaState state, lua_State* L, const FPolygons &polygons, const int polygonOrientations[])
{
	int count = polygons.size();
	lua_createtable(L, count, 0); // [bt]
	for (int i = 0; i < count; ++i) {
		FPolygon polygon = polygons[i];

		int compCount = polygon.size();
		lua_pushinteger(L, i + 1); // [bt, i]
		lua_createtable(L, compCount * 2, 0); // [bt, i, tt]
		for (int j = 0; j < compCount; j++) {
			lua_pushnumber(L, polygon[j].mX); // [bt, i, tt, x]
			lua_rawseti (L, -2, j * 2 + 1); // [bt, i, tt]
			lua_pushnumber(L, polygon[j].mY); // [bt, i, tt, y]
			lua_rawseti (L, -2, j * 2 + 2); // [bt, i, tt]
		}
		lua_settable(L, -3); // [bt[i=tt]
	}
}

//================================================================//
// INTERSECTION GEOMS
//================================================================//

void static createIntersectionGeometries(const FPolygon &intersectionPoints, FPolygons &unionIntersectionGeometries, FPolygons &cutIntersectionGeometries, float delta)
{
	unionIntersectionGeometries.clear();
	for (int i = 0; i < intersectionPoints.size(); ++i) {
		USVec2D p = intersectionPoints[i];
		FPolygon geom;
		geom.push_back(USVec2D(p.mX - delta, p.mY + delta));
		geom.push_back(USVec2D(p.mX - delta, p.mY - delta));
		geom.push_back(USVec2D(p.mX + delta, p.mY - delta));
		geom.push_back(USVec2D(p.mX + delta, p.mY + delta));
		unionIntersectionGeometries.push_back(geom);
	}
	
	cutIntersectionGeometries.clear();
	for (int i = 0; i < intersectionPoints.size(); ++i) {
		USVec2D p = intersectionPoints[i];
		FPolygon geom;
		geom.push_back(USVec2D(p.mX - delta, p.mY + delta * 1.2));
		geom.push_back(USVec2D(p.mX - delta, p.mY - delta * 1.2));
		geom.push_back(USVec2D(p.mX + delta, p.mY - delta * 1.2));
		geom.push_back(USVec2D(p.mX + delta, p.mY + delta * 1.2));
		cutIntersectionGeometries.push_back(geom);
	}
}

//================================================================//
// OFFSETTING
//================================================================//

void static	floatToIntScale(const FPolygons &polys, Polygons &scaledPolys, float scale) {
	scaledPolys.clear();
	for (int i = 0; i < polys.size(); ++i) {
		FPolygon poly = polys[i];
		ClipperLib::Polygon scaledPoly;
		for (int j = 0; j < poly.size(); ++j) {
			USVec2D p = poly[j];
			scaledPoly.push_back(IntPoint(p.mX * scale, p.mY * scale));
		}
		scaledPolys.push_back(scaledPoly);
	}
}

void static	intToFloatScale(const Polygons &scaledPolys, FPolygons &polys, float scale) {
	polys.clear();
	for (int i = 0; i < scaledPolys.size(); ++i) {
		ClipperLib::Polygon scaledPoly = scaledPolys[i];
		FPolygon poly;
		for (int j = 0; j < scaledPoly.size(); ++j) {
			IntPoint p = scaledPoly[j];
			poly.push_back(USVec2D(p.X * scale, p.Y * scale));
		}
		polys.push_back(poly);
	}
}

static int* offsetPolyLinesToPolygons(const FPolygons &polyLines, FPolygons &unionIntersectionGeometries, FPolygons &cutIntersectionGeometries, FPolygons &unionPolygons, FPolygons &cutPolygons, float delta) {
	const float scale = 1000.0f; // scale for integers used by clipper

	Polygons scaledPolyLines, scaledUnionIntersectionGeometries, scaledCutIntersectionGeometries, scaledPolygons, scaledUnionPolygons, scaledCutPolygons;
	floatToIntScale(polyLines, scaledPolyLines, scale);
	floatToIntScale(unionIntersectionGeometries, scaledUnionIntersectionGeometries, scale);
	floatToIntScale(cutIntersectionGeometries, scaledCutIntersectionGeometries, scale);
	
 	OffsetPolyLines(scaledPolyLines, scaledPolygons, delta * scale, jtRound, etRound, 3.0);
	
	MOAIPrint("Orientation scaledIntersectionGeometries[0]: %d\n", Orientation(scaledCutIntersectionGeometries[0]));

    Clipper clpr;
    clpr.AddPolygons(scaledPolygons, ptSubject);
	clpr.AddPolygons(scaledUnionIntersectionGeometries, ptClip);
    clpr.Execute(ctUnion, scaledUnionPolygons, pftPositive, pftPositive);
	
	clpr.Clear();
    clpr.AddPolygons(scaledPolygons, ptSubject);
	clpr.AddPolygons(scaledCutIntersectionGeometries, ptClip);
    clpr.Execute(ctDifference, scaledCutPolygons, pftPositive, pftPositive);

	int* polygonOrientations = new int[scaledUnionPolygons.size()];
	for (int i = 0; i < scaledUnionPolygons.size(); ++i)
		polygonOrientations[i] = Orientation(scaledUnionPolygons[i]);

	intToFloatScale(scaledUnionPolygons, unionPolygons, 1.0f/scale);
	intToFloatScale(scaledCutPolygons, cutPolygons, 1.0f/scale);

	return polygonOrientations;
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
	
	for (int i = 0; i < polygons.size(); ++i) {
		FPolygon polygon = polygons[i];
		int count = polygon.size();
		float *comps = new float[count * 2];
		for (int j = 0; j < count; ++j) {
			comps[j * 2] = polygon[j].mX;
			comps[j * 2 + 1] = polygon[j].mY;
		}
		tessAddContour(tess, 2, comps, sizeof(float)*2, count);
	}

	tessTesselate(tess, TESS_WINDING_POSITIVE, TESS_POLYGONS, 3, 2, 0);

	MOAIPrint("Memory used: %.1f kB\n", allocated/1024.0f);
	
	const float* verts = tessGetVertices(tess);
	const int* vinds = tessGetVertexIndices(tess);
	const int* elems = tessGetElements(tess);
	const int nverts = tessGetVertexCount(tess);
	const int nelems = tessGetElementCount(tess);

	MOAIPrint("tessGetElementCount %d\n", nverts);
	MOAIPrint("tessGetVertexCount %d\n", nelems);

	for (int i = 0; i < nelems; ++i)
	{
		const int* p = &elems[i*3];
		FPolygon poly;
		for (int j = 0; j < 3 && p[j] != TESS_UNDEF; ++j)
			poly.push_back(USVec2D(verts[p[j]*2], verts[p[j]*2+1]));
		triangles.push_back(poly);
	}

	if (tess) tessDeleteTess(tess);
}

//================================================================//
// COLORIZE
//================================================================//

/*
static float pointToLineDistanceSq(const USVec2D &center, const FPolygon &line)
{
	USVec2D dir = line[1] - line[0];
	USVec2D diff = center - line[0];
	float t = diff.Dot(dir) / dir.Dot(dir);
	t = min(t, 1.0f); t = max(t, 0.0f);

	// trying to write: closest = line[0] + dir * t
	USVec2D closest = dir;
	closest.Scale(t); closest.Add(line[0]);

	float distanceSq = ZLDist::PointToPointSqrd(center, closest);
	return distanceSq;*/

	/*
	LUA:
	local dir = self.v2 - self.v1
	local diff = v - self.v1
	
	 -- t is a scalar around 0-1 indicating where on the line is closest to the sphere
	local t = V3.dot(diff, dir) / V3.dot(dir, dir)
	
	-- clamp t within [0, 1] otherwise it is outside of the line
	t = math.min(t, 1)
	t = math.max(t, 0)
	
	local closest = self.v1 + dir * t
	local distance = (closest - v):len()
	return closest, distance
	*//*
}

static u32 findClosestPolyLineColor(const USVec2D &center, const FPolygons &polyLines, const Colors &lineColors)
{
	float shortestDistanceSq = 1000000.0f; // omg, infinity almost
	u32 closestColor;
	bool first = true;
	
	int count = polyLines.size();
	for (int i = 0; i < count; ++i) {
		FPolygon line = polyLines[i];
		float distanceSq = pointToLineDistanceSq(center, line);
		if (distanceSq < shortestDistanceSq) {
			shortestDistanceSq = distanceSq;
			closestColor = lineColors[i];
		}
	}
	return closestColor;
}

static void calculateTriangleColors(const FPolygons &triangles, Colors &triangleColors, const FPolygons &polyLines, const Colors &lineColors)
{
	triangleColors.clear();

	int count = triangles.size();
	for (int i = 0; i < count; ++i) {
		FPolygon triangle = triangles[i];
		USVec2D center = triangle[0] + triangle[1] + triangle[2];
		center.Scale(1.0f/3.0f);
		u32 color = findClosestPolyLineColor(center, polyLines, lineColors);
		triangleColors.push_back(color);
	}
}*/

//================================================================//
// 3D
//================================================================//

void static writePointToStream(ZLByteStream* stream, const ZLVec3D &p)
{
	stream->Write<float>(p.mX);
	stream->Write<float>(p.mY);
	stream->Write<float>(p.mZ);
}

void static writeVertexToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p, const ZLVec3D &n, const u32 color)
{
	ZLByteStream* stream = vbo->GetStream();
	writePointToStream(stream, p);
	writePointToStream(stream, n);
	stream->Write < u32 >( color );
}

void static writeTrianglesToVBO(MOAIVertexBuffer* vbo, const FPolygons &triangles, const FPolygons &polyLines, u32 hand, float missingDimValue, float normalSign, const u32 color)
{
	ZLVec3D p3d, n;

	int count = triangles.size();
	for (int i = 0; i < triangles.size(); ++i) {
		FPolygon triangle = triangles[i];

		int compCount = triangle.size();
		for (int j = 0; j < compCount; j++) {
			USVec2D p2d = triangle[j];
			if (hand == MOAISkyways::HAND_LEFT) {
				p3d.mX = missingDimValue;
				p3d.mY = p2d.mX;

				n.mX = normalSign; n.mY = 0.0f; n.mZ = 0.0f;
			} else {
				p3d.mX = p2d.mX;
				p3d.mY = missingDimValue;

				n.mX = 0.0f; n.mY = normalSign; n.mZ = 0.0f;
			}
			p3d.mZ = p2d.mY;

			writeVertexToVBO(vbo, p3d, n, color);
		}
	}
}

void static writeTriToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p1, const ZLVec3D &p2, const ZLVec3D &p3, const u32 color)
{
	ZLVec3D n;
	n.Cross(p2 - p1, p3 - p1);
	n.Norm();
	
	writeVertexToVBO(vbo, p1, n, color);
	writeVertexToVBO(vbo, p2, n, color);
	writeVertexToVBO(vbo, p3, n, color);
}

void static	writeTopFaceToVBO(MOAIVertexBuffer* vbo, const USVec2D &p1, const USVec2D &p2, int orientation, u32 hand, float missingDimValue, float delta, const u32 color)
{
	ZLVec3D nl, nr, sr, sl; // north/left/south/right

	if (hand == MOAISkyways::HAND_LEFT) {
		nl.mX = missingDimValue - delta;
		nl.mY = p2.mX;
		nl.mZ = p2.mY;
		
		nr.mX = missingDimValue + delta;
		nr.mY = p2.mX;
		nr.mZ = p2.mY;
		
		sr.mX = missingDimValue + delta;
		sr.mY = p1.mX;
		sr.mZ = p1.mY;
		
		sl.mX = missingDimValue - delta;
		sl.mY = p1.mX;
		sl.mZ = p1.mY;
	} else {
		nl.mX = p2.mX;
		nl.mY = missingDimValue + delta;
		nl.mZ = p2.mY;
		
		nr.mX = p2.mX;
		nr.mY = missingDimValue - delta;
		nr.mZ = p2.mY;
		
		sr.mX = p1.mX;
		sr.mY = missingDimValue - delta;
		sr.mZ = p1.mY;
		
		sl.mX = p1.mX;
		sl.mY = missingDimValue + delta;
		sl.mZ = p1.mY;
	}

	writeTriToVBO(vbo, nl, sr, sl, color);
	writeTriToVBO(vbo, nl, nr, sr, color);
}

void static	writeTopFacesToVBO(MOAIVertexBuffer* vbo, const FPolygons &polygons, const int polygonOrientations[], u32 hand, float missingDimValue, float delta, const u32 color)
{
	for (int iP = 0; iP < polygons.size(); ++iP) {
		FPolygon polygon = polygons[iP];
		for (int iV = 0; iV < polygon.size(); ++iV) {
			USVec2D p1 = polygon[iV];
			USVec2D p2 = polygon[(iV + 1) % polygon.size()];
			writeTopFaceToVBO(vbo, p1, p2, polygonOrientations[iP], hand, missingDimValue, delta, color);
		}
	}
}

//================================================================//
// LUA INTERFACE
//================================================================//

int MOAISkyways::_createLegGeometry ( lua_State* L ) {
	MOAILuaState state ( L );
	if ( !state.CheckParams(1, "UTTNNN") ) return 0;
	
	MOAIVertexBuffer* vbo				= state.GetLuaObject < MOAIVertexBuffer >( 1, true );
	int lineTableIndex					= 2;
	int intersectionPointsTableIndex	= 3;
	float delta							= state.GetValue<float>(4, 0.15f);
	u32 hand							= state.GetValue < u32 >( 5, MOAISkyways::HAND_LEFT );
	float missingDimValue				= state.GetValue < float >( 6, 0.0f );
	float r								= state.GetValue < float >( 7, 0.0f );
	float g								= state.GetValue < float >( 8, 0.0f );
	float b								= state.GetValue < float >( 9, 0.0f );
	float a								= state.GetValue < float >( 10, 0.0f );

	u32 color = ZLColor::PackRGBA(r, g, b, a);
	
	FPolygons polyLines, unionIntersectionGeometries, cutIntersectionGeometries, unionPolygons, cutPolygons;
	FPolygon intersectionPoints;

	readPolyLinesFromLua(polyLines, state, L, lineTableIndex);
	readIntersectionPointsFromLua(intersectionPoints, state, L, intersectionPointsTableIndex);

	createIntersectionGeometries(intersectionPoints, unionIntersectionGeometries, cutIntersectionGeometries, delta);
	
	int* polygonOrientations = offsetPolyLinesToPolygons(polyLines, unionIntersectionGeometries, cutIntersectionGeometries, unionPolygons, cutPolygons, delta);
	
	FPolygons triangles;
	tesselatePolygons(cutPolygons, triangles);
	
	//Colors triangleColors;
	//calculateTriangleColors(triangles, triangleColors, polyLines, lineColors);

	writeTrianglesToVBO(vbo, triangles, polyLines, hand, missingDimValue + delta, 1.0f, color);
	writeTrianglesToVBO(vbo, triangles, polyLines, hand, missingDimValue - delta, -1.0f, color);

	writeTopFacesToVBO(vbo, cutPolygons, polygonOrientations, hand, missingDimValue, delta, color);
	
	pushPolygonsToLua(state, L, unionPolygons, polygonOrientations);
	
	return 1;
}

//================================================================//
// MOAIClipper
//================================================================//

//----------------------------------------------------------------//
void MOAISkyways::RegisterLuaClass ( MOAILuaState& state ) {

	luaL_Reg regTable [] = {
		{ "createLegGeometry",					_createLegGeometry },
		{ NULL, NULL }
	};

	luaL_register( state, 0, regTable );
	
	state.SetField ( -1, "HAND_LEFT",					( u32 )MOAISkyways::HAND_LEFT );
	state.SetField ( -1, "HAND_RIGHT",					( u32 )MOAISkyways::HAND_RIGHT );
}
