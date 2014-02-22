 

----------------------------------------------------------------

-- Copyright (c) 2010-2011 Zipline Games, Inc.

-- All Rights Reserved.

-- <!-- m --><a class="postlink" href="http://getmoai.com">http://getmoai.com</a><!-- m -->

----------------------------------------------------------------

 

print ( 'hello, moai!' )

 

MOAISim.openWindow ( "test", 320, 480 )

 

 

--MOAISim:timerStart()

 

 

 

local buffer_gfxQuad = MOAIGfxQuad2D.new ()

buffer_gfxQuad:setTexture ( "moai.png" )

buffer_gfxQuad:setRect ( -128, -128, 128, 128 )

buffer_gfxQuad:setUVRect ( 0, 0, 1, 1 )

 

local buffer_viewport = MOAIViewport.new ()

buffer_viewport:setSize ( 256, 256 )

buffer_viewport:setScale ( 256, 256 )

 

local buffer_layer = MOAILayer2D.new ()

buffer_layer:setViewport ( buffer_viewport )

 

local buffer_propA = MOAIProp2D.new ()

buffer_propA:setDeck ( buffer_gfxQuad )

buffer_propA:setLoc(-80,-80)

buffer_layer:insertProp ( buffer_propA )

 

 

local buffer_propB = MOAIProp2D.new ()

buffer_propB:setDeck ( buffer_gfxQuad )

buffer_layer:insertProp ( buffer_propB )

buffer_propB:setLoc(80,80)

 

 

local buffer_frame = MOAIFrameBufferTexture.new ()

 

buffer_frame:setRenderTable ({ buffer_layer })

buffer_frame:init ( 256, 256 )

 

buffer_frame:setClearDepth(true)

buffer_frame:setClearColor() --MUST BE CLEAR FOR ALPHA

--buffer_frame:setClearColor(1,1,0,1) --MUST RESET SOMETIMES

 

 

 

     

MOAIRenderMgr.setBufferTable ({ buffer_frame })

 

 

--************************************

--DISPLAY ON SCREEN

display_gfxQuad = MOAIGfxQuad2D.new ()

display_gfxQuad:setTexture ( buffer_frame )

display_gfxQuad:setRect ( -128, -128, 128, 128 )

display_gfxQuad:setUVRect ( 0, 0, 1, 1 )

 

display_viewport = MOAIViewport.new ()

display_viewport:setSize ( 320, 480 )

display_viewport:setScale ( 320, -480 )

 

display_layer = MOAILayer2D.new ()

display_layer:setViewport ( display_viewport )

--MOAISim.pushRenderPass ( display_layer )  --UN-COMMENT TO SEE IMAGE

 

display_prop = MOAIProp2D.new ()

display_prop:setDeck ( display_gfxQuad )

display_layer:insertProp ( display_prop )

 

 

 

--*************************************

--SAVE FRAMEBUFFER

 

local img = MOAIImage.new()

 

function afterGrab ()

img:writePNG ( "test.png" )

end

 

buffer_frame:grabNextFrame ( img, afterGrab )

 
