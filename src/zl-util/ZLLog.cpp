// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#include "pch.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <zl-util/ZLLog.h>

#ifdef ANDROID
	#include <android/log.h>
#endif

//================================================================//
// ZLLog
//================================================================//

FILE* ZLLog::CONSOLE = 0;

//----------------------------------------------------------------//
void ZLLog::Print ( cc8* format, ... ) {

	va_list args;
	va_start ( args, format );	
	
	#ifdef ANDROID
		__android_log_vprint(ANDROID_LOG_INFO,"MoaiLog", format, args);
	#else
		vprintf ( format, args );
	#endif
	
	va_end ( args );
}

//----------------------------------------------------------------//
#ifdef ANDROID
	void ZLLog::PrintFile ( ZLFILE* file, cc8* format, ... ) {
#else
	void ZLLog::PrintFile ( FILE* file, cc8* format, ... ) {
#endif

	va_list args;
	va_start ( args, format );	
	
	if ( file ) {
		#ifdef ANDROID
			zl_vfprintf ( file, format, args );
		#else
			vfprintf ( file, format, args );
		#endif
	}
	else {
		#ifdef ANDROID
			__android_log_vprint(ANDROID_LOG_INFO,"MoaiLog", format, args);
		#else
			vprintf ( format, args );
		#endif
	}
	
	va_end ( args );
}

//----------------------------------------------------------------//
#ifdef ANDROID
	void ZLLog::PrintFileV ( ZLFILE* file, cc8* format, va_list args ) {
#else
	void ZLLog::PrintFileV ( FILE* file, cc8* format, va_list args ) {
#endif
	
	if ( file ) {
		#ifdef ANDROID
			zl_vfprintf ( file, format, args );
		#else
			vfprintf ( file, format, args );
		#endif
	}
	else {
		#ifdef ANDROID
			__android_log_vprint(ANDROID_LOG_INFO,"MoaiLog", format, args);
		#else
			vprintf ( format, args );
		#endif
	}
}
