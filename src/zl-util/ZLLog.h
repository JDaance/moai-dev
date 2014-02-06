// Copyright (c) 2010-2011 Zipline Games, Inc. All Rights Reserved.
// http://getmoai.com

#ifndef ZLLOG_H
#define ZLLOG_H


//================================================================//
// ZLLog
//================================================================//
class ZLLog {



public:
	static FILE* CONSOLE;

	//----------------------------------------------------------------//
	static void	Print		( cc8* format, ... );

#ifdef ANDROID //maybe have this as USING_VFS. currently only android does the monkey patching.
	static void	PrintFile	( ZLFILE* file, cc8* format, ... );
#else
	static void	PrintFile	( FILE* file, cc8* format, ... );
#endif

#ifdef ANDROID //maybe have this as USING_VFS. currently only android does the monkey patching.
	static void	PrintFileV	( ZLFILE* file, cc8* format, va_list args );
#else
	static void	PrintFileV	( FILE* file, cc8* format, va_list args );
#endif
};

#endif
