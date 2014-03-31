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

	int result = MOAIJsonParser::Encode(L); // pushes string onto top
	if (result == 1) {
		cc8* jsonString	= state.GetValue < cc8* >( -1, NULL );

		lua_pop(L, 1); // clean up state so that any calls triggered from JS work on a clean state
		PushMessageToJs(jsonString);
	} else {
		MOAIPrint("Error posting message to JS, json serialization failed");
	}
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

	MOAILuaStrongRef& callback = MOAIApp::Get ().onJsMessageCallback;

	if ( callback ) {
		MOAIScopedLuaState L = MOAILuaRuntime::Get ().State (); // empty stack
		lua_pushstring ( L, jsonString ); // stack = S
		int result = MOAIJsonParser::Decode(L); // will decode string from stack and push table to stack, stack = T
		if (result == 1) {
			callback.PushRef( L ); // stack = TF
			lua_insert( L, -2 ); // stack = FT
			L.DebugCall ( 1, 0 ); // call
		} else {
			MOAIPrint("Error parsing message from JS, json decoding failed\n");		
		}
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