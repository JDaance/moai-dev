// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAISKYWAYS_H
#define	MOAISKYWAYS_H

class MOAISkyways :
	public MOAILuaObject {
private:

	//----------------------------------------------------------------//
	static int		_createLegGeometry			( lua_State* L );
	
	static int		_startProfile				( lua_State* L );
	static int		_stopAndDumpProfile			( lua_State* L );

public:
	
	DECL_LUA_SINGLETON ( MOAISkyways )

	//----------------------------------------------------------------//
	enum {
		HAND_LEFT,
		HAND_RIGHT
	};
	
	//----------------------------------------------------------------//
	void				RegisterLuaClass		( MOAILuaState& state );
};

#endif