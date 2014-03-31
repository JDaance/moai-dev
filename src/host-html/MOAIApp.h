// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef	MOAIAPP_H
#define	MOAIAPP_H

#include "moai-core/headers.h"

class MOAIApp :
	public MOAIGlobalClass < MOAIApp, MOAILuaObject > {
private:
	
	MOAILuaMemberRef		onJsMessageCallback;

	//----------------------------------------------------------------//
	static int		_setOnJsMessageCallback					( lua_State* L );
	static int		_postMessageToJs						( lua_State* L );

public:
	
	DECL_LUA_SINGLETON ( MOAIApp )
	
	//----------------------------------------------------------------//
				MOAIApp														();
				~MOAIApp													();
	void		OnInit														();
	void		Reset														();

	static void HandleMessageFromJs											( const char* jsonString );
	
	void		RegisterLuaClass											( MOAILuaState& state );
};

#endif
