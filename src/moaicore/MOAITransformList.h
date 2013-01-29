// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAITRANSFORMLIST_H
#define	MOAITRANSFORMLIST_H

#include <moaicore/MOAILua.h>

class MOAITransform;

//================================================================//
// MOAITransformList
//================================================================//
/**	@name	MOAITransformList
 @text	Array of transforms (for instance, a set of Bones in a skeletal animation).
 */
class MOAITransformList :
public virtual MOAINode {
private:
	
	USLeanArray < MOAITransform* > mTransforms;
	
	//----------------------------------------------------------------//
	static int		_getTransform			( lua_State* L );
	static int		_reserve				( lua_State* L );
	static int		_setTransform			( lua_State* L );
	
	//----------------------------------------------------------------//
	void			Clear					();
	
public:
	
	DECL_LUA_FACTORY ( MOAITransformList )
	
	//----------------------------------------------------------------//
	MOAITransformList		();
	~MOAITransformList		();
	MOAITransform*	GetTransform			( u32 idx );
	void			RegisterLuaClass		( MOAILuaState& state );
	void			RegisterLuaFuncs		( MOAILuaState& state );
	void			Reserve					( u32 total, bool allocate );
	
	void			SetTransform			( u32 idx, MOAITransform* transform );
	u32				Size					();
};

#endif
