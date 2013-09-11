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

struct FloatPoint {
public:
  float X;
  float Y;
  FloatPoint(float x = 0, float y = 0): X(x), Y(y) {};
};

typedef std::vector< FloatPoint > FPolygon;
typedef std::vector< FPolygon > FPolygons;

//================================================================//
// LUA TABLE TRANSLATE
//================================================================//

void static readPolyLinesFromLua(FPolygons &polyLines, MOAILuaState state, lua_State* L, int tableIndex)
{
	int length = luaL_getn(L, tableIndex);
	
	float x, y;

	u32 lineCount = 0;
	lua_pushnil ( L );
    while ( lua_next ( L, tableIndex ) != 0 ) {
		lua_pushnil ( L );
		u32 compCount = 0;
		FPolygon polyLine;
		while ( lua_next ( L, -2 ) != 0 ) {
			if ( compCount % 2 == 0 ) {
				x = state.GetValue < float >( -1, 0.0f );
			} else {
				y = state.GetValue < float >( -1, 0.0f );
				polyLine.push_back(FloatPoint(x, y));
			}
			++compCount;
			lua_pop ( L, 1 );
		}
		polyLines.push_back(polyLine);
		++lineCount;
		lua_pop ( L, 1 );
	}
}

void static pushTrianglesToLua(MOAILuaState state, lua_State* L, const FPolygons &triangles)
{
	int count = triangles.size();
	lua_createtable(L, count, 0); // [bt]
	for (int i = 0; i < triangles.size(); ++i) {
		FPolygon triangle = triangles[i];

		int compCount = triangle.size();
		lua_pushinteger(L, i + 1); // [bt, i]
		lua_createtable(L, compCount * 2, 0); // [bt, i, tt]
		for (int j = 0; j < compCount; j++) {
			lua_pushnumber(L, triangle[j].X); // [bt, i, tt, x]
			lua_rawseti (L, -2, j * 2 + 1); // [bt, i, tt]
			lua_pushnumber(L, triangle[j].Y); // [bt, i, tt, y]
			lua_rawseti (L, -2, j * 2 + 2); // [bt, i, tt]
		}
		lua_settable(L, -3); // [bt[i=tt]
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
			FloatPoint p = poly[j];
			scaledPoly.push_back(IntPoint(p.X * scale, p.Y * scale));
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
			poly.push_back(FloatPoint(p.X * scale, p.Y * scale));
		}
		polys.push_back(poly);
	}
}

static int* offsetPolyLinesToPolygons(const FPolygons &polyLines, FPolygons &polygons, float delta) {
	const float scale = 1000.0f; // scale for integers used by clipper

	Polygons scaledPolyLines, scaledPolygons;
	floatToIntScale(polyLines, scaledPolyLines, scale);
	
 	OffsetPolyLines(scaledPolyLines, scaledPolygons, delta * scale, jtRound, etRound, 3.0);

	int* polygonOrientations = new int[scaledPolygons.size()];
	for (int i = 0; i < scaledPolygons.size(); ++i)
		polygonOrientations[i] = Orientation(scaledPolygons[i]);

	intToFloatScale(scaledPolygons, polygons, 1.0f/scale);

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
			comps[j * 2] = polygon[j].X;
			comps[j * 2 + 1] = polygon[j].Y;
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
			poly.push_back(FloatPoint(verts[p[j]*2], verts[p[j]*2+1]));
		triangles.push_back(poly);
	}

	if (tess) tessDeleteTess(tess);
}

//================================================================//
// 3D
//================================================================//

void static writePointToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p)
{
	ZLByteStream* stream = vbo->GetStream();
	stream->Write<float>(p.mX);
	stream->Write<float>(p.mY);
	stream->Write<float>(p.mZ);
}

void static writeTrianglesToVBO(MOAIVertexBuffer* vbo, const FPolygons &triangles, u32 hand, float missingDimValue, float normalSign)
{
	ZLVec3D p, n;

	int count = triangles.size();
	for (int i = 0; i < triangles.size(); ++i) {
		FPolygon triangle = triangles[i];

		int compCount = triangle.size();
		for (int j = 0; j < compCount; j++) {
			if (hand == MOAISkyways::HAND_LEFT) {
				p.mX = missingDimValue;
				p.mY = triangle[j].X;

				n.mX = normalSign; n.mY = 0.0f; n.mZ = 0.0f;
			} else {
				p.mX = triangle[j].X;
				p.mY = missingDimValue;

				n.mX = 0.0f; n.mY = normalSign; n.mZ = 0.0f;
			}
			p.mZ = triangle[j].Y;

			writePointToVBO(vbo, p);
			writePointToVBO(vbo, n);
		}
	}
}

void static writeTriToVBO(MOAIVertexBuffer* vbo, const ZLVec3D &p1, const ZLVec3D &p2, const ZLVec3D &p3)
{
	ZLVec3D n;
	n.Cross(p2 - p1, p3 - p1);
	n.Norm();

	writePointToVBO(vbo, p1);
	writePointToVBO(vbo, n);
	writePointToVBO(vbo, p2);
	writePointToVBO(vbo, n);
	writePointToVBO(vbo, p3);
	writePointToVBO(vbo, n);
}

void static	writeTopFaceToVBO(MOAIVertexBuffer* vbo, const FloatPoint &p1, const FloatPoint &p2, int orientation, u32 hand, float missingDimValue, float delta)
{
	ZLVec3D nl, nr, sr, sl; // north/left/south/right

	if (hand == MOAISkyways::HAND_LEFT) {
		nl.mX = missingDimValue - delta;
		nl.mY = p2.X;
		nl.mZ = p2.Y;
		
		nr.mX = missingDimValue + delta;
		nr.mY = p2.X;
		nr.mZ = p2.Y;
		
		sr.mX = missingDimValue + delta;
		sr.mY = p1.X;
		sr.mZ = p1.Y;
		
		sl.mX = missingDimValue - delta;
		sl.mY = p1.X;
		sl.mZ = p1.Y;
	} else {
		nl.mX = p2.X;
		nl.mY = missingDimValue + delta;
		nl.mZ = p2.Y;
		
		nr.mX = p2.X;
		nr.mY = missingDimValue - delta;
		nr.mZ = p2.Y;
		
		sr.mX = p1.X;
		sr.mY = missingDimValue - delta;
		sr.mZ = p1.Y;
		
		sl.mX = p1.X;
		sl.mY = missingDimValue + delta;
		sl.mZ = p1.Y;
	}

	writeTriToVBO(vbo, nl, sr, sl);
	writeTriToVBO(vbo, nl, nr, sr);
}

void static	writeTopFacesToVBO(MOAIVertexBuffer* vbo, const FPolygons &polygons, const int polygonOrientations[], u32 hand, float missingDimValue, float delta)
{
	for (int iP = 0; iP < polygons.size(); ++iP) {
		FPolygon polygon = polygons[iP];
		for (int iV = 0; iV < polygon.size(); ++iV) {
			FloatPoint p1 = polygon[iV];
			FloatPoint p2 = polygon[(iV + 1) % polygon.size()];
			writeTopFaceToVBO(vbo, p1, p2, polygonOrientations[iP], hand, missingDimValue, delta);
		}
	}
}

//================================================================//
// LUA INTERFACE
//================================================================//

int MOAISkyways::_createLegGeometry ( lua_State* L ) {
	MOAILuaState state ( L );
	if ( !state.CheckParams(1, "UTNNN") ) return 0;
	
	MOAIVertexBuffer* vbo		= state.GetLuaObject < MOAIVertexBuffer >( 1, true );
	int tableIndex				= 2;
	float delta					= state.GetValue<float>(3, 0.15f);
	u32 hand					= state.GetValue < u32 >( 4, MOAISkyways::HAND_LEFT );
	float missingDimValue		= state.GetValue < float >( 5, 0.0f );

	FPolygons polyLines, polygons;
	readPolyLinesFromLua(polyLines, state, L, tableIndex);
	
	int* polygonOrientations = offsetPolyLinesToPolygons(polyLines, polygons, delta);
	
	FPolygons triangles;
	tesselatePolygons(polygons, triangles);

	writeTrianglesToVBO(vbo, triangles, hand, missingDimValue + delta, 1.0f);
	writeTrianglesToVBO(vbo, triangles, hand, missingDimValue - delta, -1.0f);

	writeTopFacesToVBO(vbo, polygons, polygonOrientations, hand, missingDimValue, delta);
	
	pushTrianglesToLua(state, L, triangles);
	
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
