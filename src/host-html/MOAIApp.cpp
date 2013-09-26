#include "MOAIApp.h"
#include "HtmlHost.h"


//----------------------------------------------------------------//
/**	@name	_setOnJsMessageCallback
	@text	Set a callback to receieve message from javascript.
	
	@in		function handler
	@out	nil
*/
int MOAIApp::_setOnJsMessageCallback ( lua_State* L ) {
	MOAILuaState state ( L );
	
	MOAIApp::Get ().onJsMessageCallback.SetStrongRef ( state, 1 );
	
	return 0;
}

//----------------------------------------------------------------//
/**	@name	_postMessageToJs
	@text	Send a message to javascript.
	
	@in		function handler
	@out	nil
*/
int MOAIApp::_postMessageToJs ( lua_State* L ) {
	MOAILuaState state ( L );
	
	cc8* message	= state.GetValue < cc8* >( 1, NULL );

	PushMessageToJs(message);
	
	return 0;
}
 

//----------------------------------------------------------------//
MOAIApp::MOAIApp () {

	RTTI_SINGLE ( MOAILuaObject )
}

//----------------------------------------------------------------//
MOAIApp::~MOAIApp () {
}

//----------------------------------------------------------------//
void MOAIApp::OnInit () {
}

//----------------------------------------------------------------//
void MOAIApp::Reset () {
	onJsMessageCallback.Clear();
}

//----------------------------------------------------------------//
void MOAIApp::HandleMessageFromJs ( const char* jsonString ) {

	MOAILuaRef& callback = MOAIApp::Get ().onJsMessageCallback;

	if ( callback ) {
	
		MOAIScopedLuaState L = callback.GetSelf ();
		
		lua_pushstring ( L, jsonString );		
		
		L.DebugCall ( 1, 0 );
	}
}

//----------------------------------------------------------------//
void MOAIApp::RegisterLuaClass ( MOAILuaState& state ) {
	
	luaL_Reg regTable[] = {
		{ "_setOnJsMessageCallback",			_setOnJsMessageCallback },
		{ "_postMessageToJs",					_postMessageToJs },
		
		{ NULL, NULL }
	};

	luaL_register( state, 0, regTable );
}