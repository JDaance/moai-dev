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
  },

  PushMessageToJs: function(jsonString) {
    if (window.RecieveLuaMessage) {
      window.RecieveLuaMessage(Module.Pointer_stringify(jsonString));
    }
  }
}

mergeInto(LibraryManager.library, LibraryMOAI);