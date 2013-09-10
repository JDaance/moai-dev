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
// OFFSETTING
//================================================================//

Polygons static readPolyLinesFromLua(MOAILuaState state, lua_State* L, int tableIndex, float scale)
{
	Polygons polyLines;

	int length = luaL_getn(L, tableIndex);
	
	int x = 0, y = 0;

	u32 lineCount = 0;
	lua_pushnil ( L );
    while ( lua_next ( L, tableIndex ) != 0 ) {
		lua_pushnil ( L );
		u32 compCount = 0;
		while ( lua_next ( L, -2 ) != 0 ) {
			if ( compCount % 2 == 0 ) {
				x = (int)(state.GetValue < float >( -1, 0.0f ) * scale);
			} else {
				y = (int)(state.GetValue < float >( -1, 0.0f ) * scale);
				polyLines[lineCount].push_back(IntPoint(x, y));
			}
			++compCount;
			lua_pop ( L, 1 );
		}
		++lineCount;
		lua_pop ( L, 1 );
	}

	return polyLines;
}

void static pushTrianglesToLua(MOAILuaState state, lua_State* L, Polygons triangles, float scale)
{
	int count = triangles.size();
	lua_createtable(L, count, 0); // [bt]
	for (int i = 0; i < triangles.size(); ++i) {
		ClipperLib::Polygon triangle = triangles[i];
		MOAIPrint("Polygon orientation: %d\n", Orientation(triangle));

		int compCount = triangle.size();
		lua_pushinteger(L, i + 1); // [bt, i]
		lua_createtable(L, compCount * 2, 0); // [bt, i, tt]
		for (int i = 0; i < compCount; i++) {
			lua_pushnumber(L, triangle[i].X / scale); // [bt, i, tt, x]
			lua_rawseti (L, -2, i * 2 + 1); // [bt, i, tt]
			lua_pushnumber(L, triangle[i].Y / scale); // [bt, i, tt, y]
			lua_rawseti (L, -2, i * 2 + 2); // [bt, i, tt]
		}
		lua_settable(L, -3); // [bt[i=tt]
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

int static tesselatePolygons(Polygons &polygons, Polygons &out) {
	int allocated = 0;
	TESSalloc ma;
	memset(&ma, 0, sizeof(ma));
	ma.memalloc = stdAlloc;
	ma.memfree = stdFree;
	ma.userData = (void*)&allocated;
	ma.extraVertices = 256; // realloc not provided, allow 256 extra vertices.

	TESStesselator* tess = tessNewTess(&ma);
	
	for (int i = 0; i < polygons.size(); ++i) {
		ClipperLib::Polygon polygon = polygons[i];
		int count = polygon.size();
		float *comps = new float[count * 2];
		for (int j = 0; j < count; ++j) {
			comps[j * 2] = polygon[j].X;
			comps[j * 2 + 1] = polygon[j].Y;
		}
		tessAddContour(tess, 2, comps, sizeof(float)*2, count);
	}
	if (!tessTesselate(tess, TESS_WINDING_POSITIVE, TESS_POLYGONS, 3, 2, 0))
		return -1;
	MOAIPrint("Memory used: %.1f kB\n", allocated/1024.0f);
	
	const float* verts = tessGetVertices(tess);
	const int* vinds = tessGetVertexIndices(tess);
	const int* elems = tessGetElements(tess);
	const int nverts = tessGetVertexCount(tess);
	const int nelems = tessGetElementCount(tess);

	MOAIPrint("tessGetElementCount %d\n", nverts);
	MOAIPrint("tessGetVertexCount %d\n", nelems);

	out.clear();

	for (int i = 0; i < nelems; ++i)
	{
		const int* p = &elems[i*3];
		ClipperLib::Polygon poly;
		for (int j = 0; j < 3 && p[j] != TESS_UNDEF; ++j)
			poly.push_back(IntPoint(verts[p[j]*2], verts[p[j]*2+1]));
		out.push_back(poly);
	}

	if (tess) tessDeleteTess(tess);
	return 0;
}

//================================================================//
// 3D
//================================================================//

void static writeTrianglesToVBO(MOAIVertexBuffer* vbo, Polygons triangles, float delta, float scale, u32 hand, float missingDimValue)
{
	float x, y, z;
	ZLByteStream* stream = vbo->GetStream();

	int count = triangles.size();
	for (int i = 0; i < triangles.size(); ++i) {
		ClipperLib::Polygon triangle = triangles[i];
		MOAIPrint("Polygon orientation: %d\n", Orientation(triangle));

		int compCount = triangle.size();
		for (int i = 0; i < compCount; i++) {
			if (hand == MOAISkyways::HAND_LEFT) {
				x = missingDimValue;
				y = triangle[i].X / scale;
			} else {
				x = triangle[i].X / scale;
				y = missingDimValue;
			}
			z = triangle[i].Y / scale;

			stream->Write<float>(x);
			stream->Write<float>(y);
			stream->Write<float>(z);
		}
	}
}

//================================================================//
// LUA INTERFACE
//================================================================//

int MOAISkyways::_createLegGeometry ( lua_State* L ) {
	MOAILuaState state ( L );
	if ( !state.CheckParams(1, "UTN") ) return 0;

	const float scale = 1000.0f; // scale for integers used by clipper
	
	MOAIVertexBuffer* vbo		= state.GetLuaObject < MOAIVertexBuffer >( 1, true );
	int tableIndex				= 2;
	float delta					= state.GetValue<float>(3, 0.15f) * scale;
	u32 hand					= state.GetValue < u32 >( 4, MOAISkyways::HAND_LEFT );
	float missingDimValue		= state.GetValue < float >( 5, 0.0f );

	Polygons polyLines = readPolyLinesFromLua(state, L, tableIndex, scale);
	Polygons polygons;
	
 	OffsetPolyLines(polyLines, polygons, delta, jtRound, etRound, 3.0);
	
	Polygons triangles;
	tesselatePolygons(polygons, triangles);

	writeTrianglesToVBO(vbo, triangles, delta, scale, hand, missingDimValue + delta);
	writeTrianglesToVBO(vbo, triangles, delta, scale, hand, missingDimValue - delta);
	
	pushTrianglesToLua(state, L, triangles, scale);
	
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
