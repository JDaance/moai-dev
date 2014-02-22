/*
 * Copyright (c) Xtremics Ltd All rights reserved.
 *
 * This software is the confidential and proprietary information of Xtremics
 * ("Confidential Information"). You shall not disclose such Confidential
 * Information and shall use it only in accordance with the terms of the
 * license agreement you entered into with Xtremics.
 */
#include "pch.h"

#include <jni.h>

#include <moai-android/moaiext-jni.h>
#include <moai-android/MOAISimpleSoundSystemAndroid.h>

extern JavaVM* jvm;

int MOAISimpleSoundSystemAndroid::_initialize(lua_State* L) {
	MOAILuaState state(L);
	
	int maxStreams = lua_tointeger(state, 1);
	
	JNI_GET_ENV (jvm, env);
	
	jclass audioHandler = env->FindClass("com/ziplinegames/moai/MoaiSimpleAudio");
	if (audioHandler == NULL) {
		ZLLog::Print ("MOAISimpleSoundSystemAndroid: Unable to find java class %s", "com/ziplinegames/moai/MoaiSimpleAudio");
	} else {
		jmethodID init = env->GetStaticMethodID (audioHandler, "init", "(I)V");
		if (init == NULL) {
			ZLLog::Print ("MOAISimpleSoundSystemAndroid: Unable to find static java method %s", "init");
		} else {
			env->CallStaticVoidMethod(audioHandler, init, maxStreams);
		}
	}
	
	return 0;
}

MOAISimpleSoundSystemAndroid::MOAISimpleSoundSystemAndroid() {
	RTTI_SINGLE(MOAILuaObject)
}

MOAISimpleSoundSystemAndroid::~MOAISimpleSoundSystemAndroid() {
}

void MOAISimpleSoundSystemAndroid::RegisterLuaClass(MOAILuaState& state) {
	
	luaL_Reg regTable[] = {
		{"initialize", _initialize},
		{NULL, NULL}
	};

	luaL_register(state, 0, regTable);
}
