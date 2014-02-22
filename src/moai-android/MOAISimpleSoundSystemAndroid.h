#ifndef	MOAISIMPLESOUNSYSTEMANDROID_H
#define	MOAISIMPLESOUNSYSTEMANDROID_H

#include <moai-core/headers.h>

class MOAISimpleSoundSystemAndroid : 
	public MOAIGlobalClass < MOAISimpleSoundSystemAndroid, MOAILuaObject > {

private:
	static int _initialize(lua_State* L);

public:
	DECL_LUA_SINGLETON(MOAISimpleSoundSystemAndroid);
	
	MOAISimpleSoundSystemAndroid();
	~MOAISimpleSoundSystemAndroid();
	void RegisterLuaClass(MOAILuaState& state);
};

#endif