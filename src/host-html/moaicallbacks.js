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

  PushMessageToJs: function(jsonString) {
    window.RecieveLuaMessage(Module.Pointer_stringify(jsonString));
  }
}

mergeInto(LibraryManager.library, LibraryMOAI);