#include "MOAIApp.h"
#include "HtmlHost.h"
#include "moai-util/MOAIJsonParser.h"


//----------------------------------------------------------------//
/**	@name	_setOnJsMessageCallback
	@text	Set a callback to receieve message from javascript.
	
	@in		function handler
	@out	nil
*/
int MOAIApp::_setOnJsMessageCallback ( lua_State* L ) {
	MOAILuaState state ( L );
	
	MOAIApp::Get ().onJsMessageCallback.SetRef ( state, 1 );
	
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

	// position 1 is message object
	cc8* string	= state.GetValue < cc8* >( -1, NULL );
	PushMessageToJs(string);
		
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
void MOAIApp::HandleMessageFromJs ( const char* string ) {

	MOAILuaStrongRef& callback = MOAIApp::Get ().onJsMessageCallback;

	if ( callback ) {
		MOAIScopedLuaState L = MOAILuaRuntime::Get ().State (); // empty stack
		lua_pushstring ( L, string ); // stack = S
		callback.PushRef( L ); // stack = SF
		lua_insert( L, -2 ); // stack = FS
		L.DebugCall ( 1, 0 ); // call, stack =
	}
}

//----------------------------------------------------------------//
void MOAIApp::RegisterLuaClass ( MOAILuaState& state ) {
	
	luaL_Reg regTable[] = {
		{ "setOnJsMessageCallback",			_setOnJsMessageCallback },
		{ "postMessageToJs",					_postMessageToJs },
		
		{ NULL, NULL }
	};

	luaL_register( state, 0, regTable );
}