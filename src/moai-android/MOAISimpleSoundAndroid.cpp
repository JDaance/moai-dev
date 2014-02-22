#include "pch.h"

#include <jni.h>

#include <moai-android/moaiext-jni.h>
#include <moai-android/MOAISimpleSoundAndroid.h>

extern JavaVM* jvm;

//================================================================//
// local
//================================================================//

//----------------------------------------------------------------//
/**	@name	load
	@text	Loads a sound from disk.
	
	@in		MOAISimpleSoundAndroid self
	@in		string filename
	@out	nil
*/
int MOAISimpleSoundAndroid::_load ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAISimpleSoundAndroid, "U" )

	self->Unload ();

	cc8* filename = state.GetValue < cc8* >( 2, "" );

	JNI_GET_ENV ( jvm, env );
	
	JNI_GET_JSTRING ( filename, jfilename );
	
	jclass audioHandler = env->FindClass ( "com/ziplinegames/moai/MoaiSimpleAudio" );
	if ( audioHandler == NULL ) {
		ZLLog::Print ( "MOAISimpleAudioAndroid: Unable to find java class %s", "com/ziplinegames/moai/MoaiSimpleAudio" );
	} else {
		jmethodID loadSound = env->GetStaticMethodID ( audioHandler, "loadSound", "(Ljava/lang/String;)I" );
		if ( loadSound == NULL ) {
			ZLLog::Print ("MOAISimpleAudioAndroid: Unable to find static java method %s", "loadSound");
		} else {
			int soundId = env->CallStaticIntMethod ( audioHandler, loadSound, jfilename );
			self->mSoundId = soundId;
		}
	}
	return 0;
}

//----------------------------------------------------------------//
/**	@name	play
	@text	Play the sound.
	
	@in		MOAISimpleSoundAndroid self
	@out	nil
*/
int MOAISimpleSoundAndroid::_play ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAISimpleSoundAndroid, "U" )
	
	if ( self->mSoundId ) {

		JNI_GET_ENV ( jvm, env );
		
		jclass audioHandler = env->FindClass ( "com/ziplinegames/moai/MoaiSimpleAudio" );
		jmethodID playSound = env->GetStaticMethodID ( audioHandler, "playSound", "(II)I" );
		env->CallStaticIntMethod ( audioHandler, playSound, self->mSoundId, self->mLooping );
	}
	return 0;
}

//----------------------------------------------------------------//
/**	@name	setLooping
	@text	Set or clear the looping status of the sound.
	
	@in		MOAISimpleSoundAndroid self
	@opt	boolean looping		Default value is 'false.'
	@out	nil
*/
int MOAISimpleSoundAndroid::_setLooping ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAISimpleSoundAndroid, "U" )
	
	bool loop = state.GetValue < bool >( 2, false );
	self->mLooping = loop;
	return 0;
}

//----------------------------------------------------------------//
/**	@name	setLoopPoints
	@text	Sets the start and end looping positions for the sound
	
	@in		MOAISimpleSoundAndroid self
	@in		double startTime
	@in		double endTime
	
	@out	nil
*/
int MOAISimpleSoundAndroid::_setLoopPoints ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAISimpleSoundAndroid, "U" )
	
	if ( self->mSoundId ) {
		double startTime = state.GetValue < double >( 2, 0.0 );
		double endTime = state.GetValue < double >( 3, 0.0 );
		//self->mSound->setLoopPoints ( startTime, endTime );
	}
	return 0;
}

//----------------------------------------------------------------//
/**	@name	setVolume
	@text	Sets the volume of the sound.
	
	@in		MOAISimpleSoundAndroid self
	@opt	boolean volume			Default value is 0.
	@out	nil
*/
int MOAISimpleSoundAndroid::_setVolume ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAISimpleSoundAndroid, "U" )
	
	float volume = state.GetValue < float >( 2, 0.0f );
	self->SetVolume ( volume );

	return 0;
}

//----------------------------------------------------------------//
/**	@name	stop
	@text	Stops the sound from playing.
	
	@in		MOAISimpleSoundAndroid self
	@out	nil
*/
int MOAISimpleSoundAndroid::_stop ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAISimpleSoundAndroid, "U" )
	
	if ( self->mSoundId ) {

		JNI_GET_ENV ( jvm, env );

		jclass audioHandler = env->FindClass ( "com/ziplinegames/moai/MoaiSimpleAudio" );
		jmethodID stopSound = env->GetStaticMethodID ( audioHandler, "stopSound", "(I)V" );
		env->CallStaticVoidMethod ( audioHandler, stopSound, self->mSoundId );
	}
	return 0;
}

//================================================================//
// MOAISimpleSoundAndroid
//================================================================//

//----------------------------------------------------------------//
bool MOAISimpleSoundAndroid::ApplyAttrOp ( u32 attrID, MOAIAttrOp& attrOp, u32 op ) {

	if ( MOAISimpleSoundAndroidAttr::Check ( attrID )) {
		attrID = UNPACK_ATTR ( attrID );

		if ( attrID == ATTR_VOLUME ) {
			this->SetVolume ( attrOp.Apply ( this->mVolume, op, MOAIAttrOp::ATTR_READ_WRITE ));
			return true;
		}
	}
	return false;
}

//----------------------------------------------------------------//
MOAISimpleSoundAndroid::MOAISimpleSoundAndroid () :
	mLooping ( false ),
	mSoundId ( 0 ),
	mVolume ( 1 ) {

	RTTI_SINGLE ( MOAINode )
}

//----------------------------------------------------------------//
MOAISimpleSoundAndroid::~MOAISimpleSoundAndroid () {

	this->Unload ();
}
//----------------------------------------------------------------//
void MOAISimpleSoundAndroid::RegisterLuaClass ( MOAILuaState& state ) {

	MOAINode::RegisterLuaClass ( state );

	state.SetField ( -1, "ATTR_VOLUME", MOAISimpleSoundAndroidAttr::Pack ( ATTR_VOLUME ));
}

//----------------------------------------------------------------//
void MOAISimpleSoundAndroid::RegisterLuaFuncs ( MOAILuaState& state ) {

	MOAINode::RegisterLuaFuncs ( state );

	luaL_Reg regTable [] = {
		{ "load",				_load },
		{ "play",				_play },
		{ "setLooping",			_setLooping },
		{ "setLoopPoints",		_setLoopPoints },
		{ "setVolume",			_setVolume },
		{ "stop",				_stop },
		{ NULL, NULL }
	};

	luaL_register ( state, 0, regTable );
}

//----------------------------------------------------------------//
void MOAISimpleSoundAndroid::SetVolume ( float volume ) {
	this->mVolume = volume;
	
	if ( this->mSoundId ) {
		JNI_GET_ENV ( jvm, env );
		
		jclass audioHandler = env->FindClass ( "com/ziplinegames/moai/MoaiSimpleAudio" );
		jmethodID setSoundVolume = env->GetStaticMethodID ( audioHandler, "setSoundVolume", "(IFF)V" );
		env->CallStaticVoidMethod ( audioHandler, setSoundVolume, this->mSoundId, volume, volume );
	}
}

//----------------------------------------------------------------//
void MOAISimpleSoundAndroid::Unload () {

	if ( this->mSoundId ) {
		JNI_GET_ENV ( jvm, env );
		
		jclass audioHandler = env->FindClass ( "com/ziplinegames/moai/MoaiSimpleAudio" );
		jmethodID unloadSound = env->GetStaticMethodID (audioHandler, "unloadSound", "(I)V");
		env->CallStaticVoidMethod(audioHandler, unloadSound, this->mSoundId);

		this->mSoundId = 0;
	}
}
