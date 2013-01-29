// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"
#include <moaicore/MOAIGfxDevice.h>
#include <moaicore/MOAITransformList.h>
#include <moaicore/MOAITransform.h>
#include <moaicore/MOAITextureBase.h>

//================================================================//
// local
//================================================================//

//----------------------------------------------------------------//
/**	@name	getTransform
	@text	Set or clears a transform for a given index. Note: If you leave this uninitialized,
			and try to use this as a skeleton for a MOAIProp, badness will ensue!
 
	@in		MOAITransformList self
	@in		number index
	@opt	MOAITransform transform		Default value is nil.
	@out	nil
*/
int MOAITransformList::_getTransform( lua_State *L ) {
	
	MOAI_LUA_SETUP ( MOAITransformList, "UN" )
	
	u32 idx						= state.GetValue < u32 >( 2, 1 ) - 1;
	
	MOAITransform* transform = self->GetTransform(idx);
	if(	transform ) {
		state.Push( transform );
	}
	else {
		lua_pushnil(L);
	}
	
	return 1;
}

//----------------------------------------------------------------//
/**	@name	reserve
	@text	Reserve (and either clear or allocate) slots for transforms. This will
			clear out any existing transforms before resizing the underlying array.
 
	@in		MOAITransformList self
	@opt	number total			Default value is 0.
	@opt	boolean allocate		Allocate new MOAITransforms for each new slot.
	@out	nil
*/
int MOAITransformList::_reserve ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAITransformList, "U" )
	
	u32 total = state.GetValue < u32 >( 2, 0 );
	bool allocate = state.GetValue <bool>( 3, false );
	
	self->Reserve ( total, allocate );
	
	return 0;
}

//----------------------------------------------------------------//
/**	@name	setTransform
	@text	Set or clears a transform for a given index. Note: If you leave this uninitialized,
			and try to use this as a skeleton for a MOAIProp, badness will ensue!
 
	@in		MOAITransformList self
	@in		number index
	@opt	MOAITransform transform		Default value is nil.
	@out	nil
 */
int MOAITransformList::_setTransform ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAITransformList, "UN" )
	
	u32 idx						= state.GetValue < u32 >( 2, 1 ) - 1;
	MOAITransform* transform	= state.GetLuaObject < MOAITransform >( 3, true );
	
	self->SetTransform ( idx, transform );
	
	return 0;
}

//================================================================//
// MOAITransformList
//================================================================//

//----------------------------------------------------------------//
void MOAITransformList::Clear () {
}

//----------------------------------------------------------------//
MOAITransformList::MOAITransformList () {
	
	RTTI_BEGIN
		RTTI_EXTEND ( MOAINode )
	RTTI_END
}

//----------------------------------------------------------------//
MOAITransformList::~MOAITransformList () {
}

//----------------------------------------------------------------//
MOAITransform* MOAITransformList::GetTransform ( u32 idx ) {
	
	if( idx >= this->mTransforms.Size() ) {
		return 0;
	}
	
	return this->mTransforms[idx];
}


////----------------------------------------------------------------//
//void MOAITransformList::SerializeIn ( MOAILuaState& state, MOAIDeserializer& serializer ) {
//	UNUSED ( state );
//	UNUSED ( serializer );
//}
//
////----------------------------------------------------------------//
//void MOAITransformList::SerializeOut ( MOAILuaState& state, MOAISerializer& serializer ) {
//	UNUSED ( state );
//	UNUSED ( serializer );
//}

//----------------------------------------------------------------//
void MOAITransformList::RegisterLuaClass ( MOAILuaState& state ) {
	UNUSED ( state );
}

//----------------------------------------------------------------//
void MOAITransformList::RegisterLuaFuncs ( MOAILuaState& state ) {
	
	luaL_Reg regTable [] = {
		{ "getTransform",			_getTransform },
		{ "reserve",				_reserve },
		{ "setTransform",			_setTransform },
		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}

//----------------------------------------------------------------//
void MOAITransformList::Reserve ( u32 total, bool allocate ) {
	
	this->Clear ();
	this->mTransforms.Init ( total );
	this->mTransforms.Fill ( 0 );
	
	if( allocate )
	{
		for (u32 i = 0; i < total; i++) {
			this->mTransforms[ i ] = new MOAITransform();
		}
	}
}

//----------------------------------------------------------------//
void MOAITransformList::SetTransform ( u32 idx, MOAITransform* transform ) {
	
	if ( idx >= this->mTransforms.Size ()) return;
	if ( this->mTransforms [ idx ] == transform ) return;
	
	this->LuaRetain ( transform );
	this->LuaRelease ( this->mTransforms [ idx ]);
	this->mTransforms [ idx ] = transform;
}

//----------------------------------------------------------------//
u32 MOAITransformList::Size() {
	return this->mTransforms.Size();
}
