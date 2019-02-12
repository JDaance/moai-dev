    Module.OpenWindowCallback=  null;

    Module.EnterFullScreenCallback = null;
    Module['SetEnterFullScreenFunc']  = function(func) {
      Module.EnterFullScreenCallback = func;
    }

    Module.ExitFullScreenCallback = null;
    Module['SetExitFullScreenFunc']  = function(func) {
      Module.ExitFullScreenCallback = func;
    }

    Module['SetOpenWindowFunc'] = function(openwindowfunc) {
      Module.OpenWindowCallback = openwindowfunc;
    }

    Module['MountIDBFSDir'] = function(dir, callback) {
      // Make a directory other than '/'
      FS.mkdir(dir);
      // Then mount with IDBFS type
      FS.mount(IDBFS, {}, dir);

      // Then sync
      FS.syncfs(true, callback);
    }

    Module['SyncFS'] = function(callback) {
      FS.syncfs(callback);
    }

