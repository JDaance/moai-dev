// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAISIMPLESOUNDANDROID_H
#define	MOAISIMPLESOUNDANDROID_H

#include <moai-core/headers.h>
#include <moai-sim/headers.h>

//================================================================//
// MOAISimpleSoundAndroid
//================================================================//
/**	@name	MOAISimpleSoundAndroid
	@text	Simple Android sound object.
	
	@attr	ATTR_VOLUME
*/
class MOAISimpleSoundAndroid :
	public virtual MOAINode {
private:

	bool 	mLooping;
	int 	mSoundId;
	float	mVolume;

	//----------------------------------------------------------------//
	static int		_load				( lua_State* L );
	static int		_play				( lua_State* L );
	static int		_setLooping			( lua_State* L );
	static int		_setLoopPoints		( lua_State* L );
	static int		_setVolume			( lua_State* L );
	static int		_stop				( lua_State* L );

public:

	DECL_LUA_FACTORY ( MOAISimpleSoundAndroid )
	DECL_ATTR_HELPER ( MOAISimpleSoundAndroid )

	enum {
		ATTR_VOLUME,
		TOTAL_ATTR,
	};

	//----------------------------------------------------------------//
	bool			ApplyAttrOp				( u32 attrID, MOAIAttrOp& attrOp, u32 op );
					MOAISimpleSoundAndroid	();
					~MOAISimpleSoundAndroid	();
	void			RegisterLuaClass		( MOAILuaState& state );
	void			RegisterLuaFuncs		( MOAILuaState& state );	
	void			SetVolume				( float volume );	
	void			Unload					();		
};

#endif
