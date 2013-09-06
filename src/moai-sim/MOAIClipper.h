// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAICLIPPER_H
#define	MOAICLIPPER_H

//================================================================//
// MOAIFileSystem
//================================================================//
/**	@name	MOAIFileSystem
	@text	Functions for manipulating the file system.
*/
class MOAIClipper :
	public MOAILuaObject {
private:

	//----------------------------------------------------------------//
	static int		_offsetPolyLines			( lua_State* L );

public:
	
	DECL_LUA_SINGLETON ( MOAIClipper )
	
	//----------------------------------------------------------------//
	void				RegisterLuaClass		( MOAILuaState& state );
};

#endif