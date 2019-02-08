var LibraryMOAI = {


  ExitFullScreen: function() {
    if (Module.ExitFullScreenCallback) {
       Module.ExitFullScreenCallback()
    }
  },

  EnterFullScreen: function() {
    if (Module.EnterFullScreenCallback) {
       Module.EnterFullScreenCallback()
    }
  },

  OpenWindowFunc: function(title,width,height) {
    var canvas;
    if (Module.OpenWindowCallback) {
      canvas = Module.OpenWindowCallback(Module.Pointer_stringify(title),width,height, function(canvas) {
        Browser.createContext(canvas,true,true);
      });
    }
    Module.canvas = canvas;
  },

  //replace fclose to notify us on file change to help with save game handling
  fclose: function(stream) {
    var fileinfo = FS.streams[stream];
    if(fileinfo && fileinfo.isWrite && Module.SaveCallback) {
      Module.SaveCallback(FS.absolutePath(fileinfo.path), fileinfo.object.contents);
    };
    _fsync(stream);
    return _close(stream);
  },

  //savegame support
  RestoreFile__deps: ['$FS'],
  RestoreFile: function(path, data) {
    //normalize the path
    var path = FS.absolutePath(path);

    var existing = FS.findObject(path);
    if (existing) {
      existing.contents = data;
    } else {
      var parts = path.split('/');
      var name = parts.pop();
      var dir = parts.join('/') || '/';
    
      FS.createPath('/',dir,true,true);
      FS.createDataFile(dir,name,data,true,true);
    }
  }
}

mergeInto(LibraryManager.library, LibraryMOAI);