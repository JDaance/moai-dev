// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"
#include <moai-sim/MOAIClipper.h>

#include <clipper/clipper.hpp>

// TODO: these are getting reintroduced somewhere; find them and kill them

#ifdef DeleteFile
	#undef DeleteFile
#endif

#ifdef RemoveDirectory
	#undef RemoveDirectory
#endif

//================================================================//
// local
//================================================================//

int MOAIClipper::_offsetPolyLines ( lua_State* L ) {
	MOAILuaState state ( L );
	if ( !state.CheckParams(1, "TN") ) return 0;

	float delta = state.GetValue<float>(1, 0.15);

	const float scale = 1000; // scale for integers used by clipper

	// read state to polylines (Polygons) and multiply by scale
	// read state to delta
	// hard code joint type n stuff

	ClipperLib::Polygons in_lines, out_lines;

	ClipperLib::OffsetPolyLines(in_lines, out_lines, delta, ClipperLib::jtMiter, ClipperLib::etButt);

	// read outlines to state and divide by scale
	// return
	
	return 0;
}

//================================================================//
// MOAIClipper
//================================================================//

//----------------------------------------------------------------//
void MOAIClipper::RegisterLuaClass ( MOAILuaState& state ) {

	luaL_Reg regTable [] = {
		{ "offsetPolyLines",					_offsetPolyLines },
		{ NULL, NULL }
	};

	luaL_register( state, 0, regTable );
}
