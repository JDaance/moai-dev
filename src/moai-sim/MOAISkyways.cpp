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
	FVertex(USVec2D v): mV(v) {};
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

void static readPolyLinesFromLua(FPolygons &polyLines, /*Colors &colors, */MOAILuaState state, lua_State* L, int tableIndex)
{
	
	int length = luaL_getn(L, tableIndex);
	
	float x1, y1, x2, y2;//, r, g, b, a;

	u32 lineCount = 0;
	lua_pushnil ( L );
	while (lineCount < length / 4) {
		x1	= popTableFloat(state, L, tableIndex);
		y1	= popTableFloat(state, L, tableIndex);
		x2	= popTableFloat(state, L, tableIndex);
		y2	= popTableFloat(state, L, tableIndex);
		
		FPolygon polyLine;
		polyLine.push_back(USVec2D(x1, y1));
		polyLine.push_back(USVec2D(x2, y2));
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

void static pushPolygonsToLua(MOAILuaState state, lua_State* L, const FPolygons &polygons)
{
	int count = polygons.size();
	lua_createtable(L, count, 0); // [bt]
	int i = 0;
	for(FPolygons::const_iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon, ++i) {
		int compCount = itPolygon->size();
		lua_pushinteger(L, i + 1); // [bt, i]
		lua_createtable(L, compCount * 2, 0); // [bt, i, tt]
		for (int j = 0; j < compCount; j++) {
			lua_pushnumber(L, (*itPolygon)[j].mV.mX); // [bt, i, tt, x]
			lua_rawseti (L, -2, j * 2 + 1); // [bt, i, tt]
			lua_pushnumber(L, (*itPolygon)[j].mV.mY); // [bt, i, tt, y]
			lua_rawseti (L, -2, j * 2 + 2); // [bt, i, tt]
		}
		lua_settable(L, -3); // [bt[i=tt]
	}
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
		geom.push_back(USVec2D(p.mX - delta, p.mY + delta * 1.2));
		geom.push_back(USVec2D(p.mX - delta, p.mY - delta * 1.2));
		geom.push_back(USVec2D(p.mX + delta, p.mY - delta * 1.2));
		geom.push_back(USVec2D(p.mX + delta, p.mY + delta * 1.2));
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
			cutGeom.push_back(USVec2D(p.mX - delta * 1.2, p.mY + delta * 1.2));
			cutGeom.push_back(USVec2D(p.mX - delta * 1.2, p.mY - delta * 1.2));
			cutGeom.push_back(USVec2D(p.mX + delta * 1.2, p.mY - delta * 1.2));
			cutGeom.push_back(USVec2D(p.mX + delta * 1.2, p.mY + delta * 1.2));
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

static void offsetPolyLinesToPolygons(const FPolygons &polyLines, FPolygons &cutIntersectionGeometries, FPolygons &cutPolygons, vector<bool> &cutPolygonOrientations, const float delta, 
	bool physics_generateSegments, const float gridSize, const vector<int> &physics_perpLegIndexes, const FPolygons &physics_unionIntersectionGeometries, const FPolygons &physics_cutIntersectionGeometries, FPolygons &physics_unionPolygons, FPolygons &physics_perpIntersectionPolygons) {
	
	const float scale = 100000.0f; // scale for integers used by clipper

	Polygons scaledPolyLines, scaledCutIntersectionGeometries, scaledPolygons, scaledCutPolygons;
	floatToIntScale(polyLines, scaledPolyLines, scale);
	
 	OffsetPolyLines(scaledPolyLines, scaledPolygons, delta * scale, jtRound, etRound, 100.0);
	
	floatToIntScale(cutIntersectionGeometries, scaledCutIntersectionGeometries, scale);

    Clipper clpr;
    clpr.AddPolygons(scaledPolygons, ptSubject);
	clpr.AddPolygons(scaledCutIntersectionGeometries, ptClip);
    clpr.Execute(ctDifference, scaledCutPolygons, pftPositive, pftPositive);

	intToFloatScale(scaledCutPolygons, cutPolygons, 1.0f/scale);

	cutPolygonOrientations.clear();
	for(Polygons::const_iterator itScaledCutPoly = scaledCutPolygons.begin(); itScaledCutPoly != scaledCutPolygons.end(); ++itScaledCutPoly) {
		cutPolygonOrientations.push_back(Orientation(*itScaledCutPoly));
	}
	
	if (physics_generateSegments) {
		Polygons physics_scaledUnionIntersectionGeometries, physics_scaledCutIntersectionGeometries, physics_scaledUnionPolygons, physics_scaledCutPolygons, physics_scaledPerpIntersectionPolygons;
		floatToIntScale(physics_unionIntersectionGeometries, physics_scaledUnionIntersectionGeometries, scale);
		floatToIntScale(physics_cutIntersectionGeometries, physics_scaledCutIntersectionGeometries, scale);

		clpr.Clear();
		clpr.AddPolygons(scaledPolygons, ptSubject);
		clpr.AddPolygons(physics_scaledUnionIntersectionGeometries, ptClip);
		clpr.Execute(ctUnion, physics_scaledUnionPolygons, pftPositive, pftPositive);
	
		clpr.Clear();
		clpr.AddPolygons(scaledPolygons, ptSubject);
		clpr.AddPolygons(physics_scaledCutIntersectionGeometries, ptClip);
		clpr.Execute(ctDifference, physics_scaledCutPolygons, pftPositive, pftPositive);

		clpr.Clear();
		clpr.AddPolygons(createPerpendicularIntersectionTestPolygons(physics_perpLegIndexes, delta, scale, gridSize), ptSubject);
		clpr.AddPolygons(physics_scaledCutPolygons, ptClip);
		clpr.Execute(ctIntersection, physics_scaledPerpIntersectionPolygons, pftPositive, pftPositive);
		
		intToFloatScale(physics_scaledUnionPolygons, physics_unionPolygons, 1.0f/scale);
		intToFloatScale(physics_scaledPerpIntersectionPolygons, physics_perpIntersectionPolygons, 1.0f/scale);
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
				if (dir2.mX > 0 || dir1.mX > 0) {
					// up
					vertex.mN = USVec2D(0.0f, 1.0f);
					isIntersectionCorner = true;
				} else if (dir2.mX < 0 || dir1.mX < 0) {
					// down
					vertex.mN = USVec2D(0.0f, -1.0f);
					isIntersectionCorner = true;
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

void static writeTrianglesToVBO(MOAIVertexBuffer* mainVbo, MOAIVertexBuffer* outlineVbo, const FPolygons &triangles, const FPolygons &polyLines, u32 hand, float missingDimValue, float normalSign, const u32 color)
{
	
	ZLVec3D p3d, n, outline_n;

	for(FPolygons::const_iterator itTriangle = triangles.begin(); itTriangle != triangles.end(); ++itTriangle) {
		int compCount = itTriangle->size();
		for (int j = 0; j < compCount; ++j) {
			// go backwards for hand left for correct culling
			FVertex v2d = hand == MOAISkyways::HAND_LEFT ? (*itTriangle)[compCount - j - 1] : (*itTriangle)[j];
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

			writeVertexToVBO(mainVbo, p3d, n, color);
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

	{
		// this tests skips faces that would face away from camera at all times
		USVec2D realLineNormal = p2 - p1;
		realLineNormal.Rotate90Anticlockwise(); // really clockwise..
		realLineNormal.Norm();

		static USVec2D backfacing(0.8, -0.6);
		if (realLineNormal.Dot(backfacing) >= 0.5)
			return;
	}

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

void static	writeTopFacesToVBO(MOAIVertexBuffer* mainVbo, MOAIVertexBuffer* outlineVbo, const FPolygons &polygons, const vector<bool> &polygonOrientations, u32 hand, float missingDimValue, float delta, const u32 color)
{
	int iP = 0;
	for(FPolygons::const_iterator itPolygon = polygons.begin(); itPolygon != polygons.end(); ++itPolygon, ++iP) {
		for (int iV = 0; iV < itPolygon->size(); ++iV) {
			FVertex v1 = (*itPolygon)[iV];
			FVertex v2 = (*itPolygon)[(iV + 1) % itPolygon->size()];
			writeTopFaceToVBO(mainVbo, outlineVbo, v1, v2, polygonOrientations[iP], hand, missingDimValue, delta, color);
		}
	}
}

//================================================================//
// LUA INTERFACE
//================================================================//

int MOAISkyways::_createLegGeometry ( lua_State* L ) {
	MOAILuaState state ( L );
	if ( !state.CheckParams(1, "UUTTNNNB") ) return 0;
	
	MOAIVertexBuffer* mainVbo					= state.GetLuaObject < MOAIVertexBuffer >( 1, true );
	MOAIVertexBuffer* outlineVbo				= state.GetLuaObject < MOAIVertexBuffer >( 2, true );
	const int lineTableIndex					= 3;
	const int intersectionPointsTableIndex		= 4;
	const float delta							= state.GetValue<float>(5, 0.15f);
	const u32 hand								= state.GetValue < u32 >( 6, MOAISkyways::HAND_LEFT );
	const float missingDimValue					= state.GetValue < float >( 7, 0.0f );
	const bool physics_generateSegments			= state.GetValue < bool >( 8, false );
	const float gridSize						= state.GetValue<float>(9, -1);
	const int physicsPerpLegIndexesTableIndex	= 10;
	const float r								= state.GetValue < float >( 11, 0.0f );
    const float g								= state.GetValue < float >( 12, 0.0f );
    const float b								= state.GetValue < float >( 13, 0.0f );
    const float a								= state.GetValue < float >( 14, 0.0f );

    const u32 color = ZLColor::PackRGBA(r, g, b, a);
	
	FPolygons polyLines, cutIntersectionGeometries, cutPolygons;
	FPolygon intersectionPoints;
	
	vector<int> physics_perpLegIndexes;
	FPolygons physics_unionIntersectionGeometries, physics_cutIntersectionGeometries, physics_unionPolygons, physics_perpIntersectionPolygons;

	readPolyLinesFromLua(polyLines, state, L, lineTableIndex);
	readIntersectionPointsFromLua(intersectionPoints, state, L, intersectionPointsTableIndex);
	if (physics_generateSegments) {
		readPerpLegIndexesFromLua(physics_perpLegIndexes, state, L, physicsPerpLegIndexesTableIndex);
	}

	createIntersectionGeometries(intersectionPoints, cutIntersectionGeometries, delta, 
		physics_generateSegments, physics_unionIntersectionGeometries, physics_cutIntersectionGeometries);
	
	vector<bool> cutPolygonOrientations;
	offsetPolyLinesToPolygons(polyLines, cutIntersectionGeometries, cutPolygons, cutPolygonOrientations, delta,
		physics_generateSegments, gridSize, physics_perpLegIndexes, physics_unionIntersectionGeometries, physics_cutIntersectionGeometries, physics_unionPolygons, physics_perpIntersectionPolygons);
	
	computeNormalsForPolygons(cutPolygons, cutPolygonOrientations);

	FPolygons triangles;
	tesselatePolygons(cutPolygons, triangles);

	// 3 = tri, 3 = arbitrary - TODO count polylines in cut for top face vertex count
	int approximateVertexCount = triangles.size() * 3 * 3 * mainVbo->GetFormat()->GetVertexSize ();
	mainVbo->Reserve(approximateVertexCount); 
	outlineVbo->Reserve(approximateVertexCount);

	// skip far side
	writeTrianglesToVBO(mainVbo, outlineVbo, triangles, polyLines, hand, missingDimValue - delta, -1.0f, color);

	writeTopFacesToVBO(mainVbo, outlineVbo, cutPolygons, cutPolygonOrientations, hand, missingDimValue, delta, color);

	MOAIPrint("Approximated %d vertices but wrote %d\n", approximateVertexCount / mainVbo->GetFormat()->GetVertexSize (), mainVbo->GetVertexCount());
	
	// TODO implement bless here instead of in LUA
	
	if (physics_generateSegments) {
		pushPolygonsToLua(state, L, physics_unionPolygons);
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
