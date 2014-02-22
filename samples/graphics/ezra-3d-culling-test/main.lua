----------------------------------------------------------------

-- Copyright (c) 2010-2011 Zipline Games, Inc.

-- All Rights Reserved.

-- <!-- m --><a class="postlink" href="http://getmoai.com">http://getmoai.com</a><!-- m -->

----------------------------------------------------------------

 

MOAISim.openWindow ( "test", 600, 600 )

 

viewport = MOAIViewport.new ()  
viewport:setSize ( 600, 600 )
viewport:setScale ( 600, 600 )   






layer = MOAILayer.new ()    
layer:setViewport ( viewport )  
MOAISim.pushRenderPass ( layer )







partition = MOAIPartition.new ()
layer:setPartition ( partition )


camera = MOAICamera.new ()  
camera:setLoc ( 0, 0, camera:getFocalLength ( 1000)) 
--camera:setOrtho(false)     
--camera:setFieldOfView(60)
camera:setFarPlane(1500)
--camera:setNearPlane (1)
             
             
             
layer:setCamera ( camera )



 
local   myTex= MOAITexture.new()

		myTex:setFilter ( MOAITexture.GL_LINEAR_MIPMAP_LINEAR ) --smooth
		myTex:load( 'moai.png', MOAIImage.PREMULTIPLY_ALPHA )
							  			
 

function makeBoxMesh ( xMin, yMin, zMin, xMax, yMax, zMax, texture )

       

        local function pushPoint ( points, x, y, z )

       

                local point = {}

                point.x = x

                point.y = y

                point.z = z

               

                table.insert ( points, point )

        end

 

        local function writeTri ( vbo, p1, p2, p3, uv1, uv2, uv3 )

               

                vbo:writeFloat ( p1.x, p1.y, p1.z )

                vbo:writeFloat ( uv1.x, uv1.y )

                vbo:writeColor32 ( 1, 1, 1 )

               

                vbo:writeFloat ( p2.x, p2.y, p2.z )

                vbo:writeFloat ( uv2.x, uv2.y )

                vbo:writeColor32 ( 1, 1, 1 )

 

                vbo:writeFloat ( p3.x, p3.y, p3.z )

                vbo:writeFloat ( uv3.x, uv3.y  )

                vbo:writeColor32 ( 1, 1, 1 )

        end

       

        local function writeFace ( vbo, p1, p2, p3, p4, uv1, uv2, uv3, uv4 )

 

                writeTri ( vbo, p1, p2, p4, uv1, uv2, uv4 )

                writeTri ( vbo, p2, p3, p4, uv2, uv3, uv4 )

        end

       

        local p = {}

       

        pushPoint ( p, xMin, yMax, zMax ) -- p1

	pushPoint ( p, xMin, yMin, zMax ) -- p2
	
	pushPoint ( p, xMax, yMin, zMax ) -- p3
	
	pushPoint ( p, xMax, yMax, zMax ) -- p4
	
	   
	
	    pushPoint ( p, xMin, yMax, zMin ) -- p5
	
	pushPoint ( p, xMin, yMin, zMin  ) -- p6
	
	pushPoint ( p, xMax, yMin, zMin  ) -- p7
	
	pushPoint ( p, xMax, yMax, zMin  ) -- p8

 

        local uv = {}

       

        pushPoint ( uv, 0, 0, 0 )

        pushPoint ( uv, 0, 1, 0 )

        pushPoint ( uv, 1, 1, 0 )

        pushPoint ( uv, 1, 0, 0 )

       

        local vertexFormat = MOAIVertexFormat.new ()

        vertexFormat:declareCoord ( 1, MOAIVertexFormat.GL_FLOAT, 3 )

        vertexFormat:declareUV ( 2, MOAIVertexFormat.GL_FLOAT, 2 )

        vertexFormat:declareColor ( 3, MOAIVertexFormat.GL_UNSIGNED_BYTE )

 

        local vbo = MOAIVertexBuffer.new ()

        vbo:setFormat ( vertexFormat )

        vbo:reserveVerts ( 36 )

       

        writeFace ( vbo, p [ 1 ], p [ 2 ], p [ 3 ], p [ 4 ], uv [ 1 ], uv [ 2 ], uv [ 3 ], uv [ 4 ])

        writeFace ( vbo, p [ 4 ], p [ 3 ], p [ 7 ], p [ 8 ], uv [ 1 ], uv [ 2 ], uv [ 3 ], uv [ 4 ])

        writeFace ( vbo, p [ 8 ], p [ 7 ], p [ 6 ], p [ 5 ], uv [ 1 ], uv [ 2 ], uv [ 3 ], uv [ 4 ])

        writeFace ( vbo, p [ 5 ], p [ 6 ], p [ 2 ], p [ 1 ], uv [ 1 ], uv [ 2 ], uv [ 3 ], uv [ 4 ])

        writeFace ( vbo, p [ 5 ], p [ 1 ], p [ 4 ], p [ 8 ], uv [ 1 ], uv [ 2 ], uv [ 3 ], uv [ 4 ])

        writeFace ( vbo, p [ 2 ], p [ 6 ], p [ 7 ], p [ 3 ], uv [ 1 ], uv [ 2 ], uv [ 3 ], uv [ 4 ])

 

        vbo:bless ()

 

        local mesh = MOAIMesh.new ()

        mesh:setTexture ( myTex )

        mesh:setVertexBuffer ( vbo )

        mesh:setPrimType ( MOAIMesh.GL_TRIANGLES )

    return mesh
end

function makeCube ( size, texture ) 
    size = size * 0.5
    return makeBoxMesh ( -size, -size, -size, size, size, size, texture )
end

--*******************************************************************************
--*******************************************************************************
--******************************************************************************* 
--******************************************************************************* 

layer:setPartitionCull2D(false)							  			

local propTable = {}

local width = 600 / 100

for i = 1 , 100 do
	for j = 1, 100 do
        local cnt =  #propTable+1
 
		local mesh = makeCube ( width, 'moai.png' )
		
		propTable[cnt] = MOAIProp.new ()
		
		propTable[cnt]:setDeck ( mesh )
		propTable[cnt]:setShader ( MOAIShaderMgr.getShader ( MOAIShaderMgr.MESH_SHADER ))
		propTable[cnt]:setCullMode ( MOAIProp.CULL_BACK )      
     	-- propTable[cnt]:moveRot ( 360, 360, 0, 20 )
		propTable[cnt]:setLoc(i*width-300,j*width-300,0) 
 	  	-- propTable[cnt]:moveLoc ( 1000,0,0 , 4, MOAIEaseType.LINEAR )				   
		partition:insertProp ( propTable[cnt] ) 
		--propTable[cnt]:setVisible(false)
	end		
end

---*************************************************************** 
---*************************************************************** 
---*************************************************************** 
---*************************************************************** 
 
local cnt = 1;

local cordinates = {
	{1000,0,0},
	{0,1000,0},
	{0,0,1000},
	
	{-1000,0,0},
	{0,-1000,0},
	{0,0,-1000},
	
	{-1000,-1000,-1000},
	{1000,1000,1000},
}

local mesh = makeCube ( 100, 'moai.png' )

local propA = MOAIProp.new ()

propA:setDeck ( mesh )          
propA:moveRot ( 360, 360, 69, 5 )   
propA:setShader ( MOAIShaderMgr.getShader ( MOAIShaderMgr.MESH_SHADER ))   
propA:setCullMode ( MOAIProp.CULL_BACK )   
propA:setLoc(0,0,0)   
partition:insertProp ( propA )

--*********************
--ACTION
local function actionDone()   
	print('***************************') 
	print('OBJECT IS NOW COMING BACK')
	print('***************************') 
	
	propA:setLoc(0,0,0)
	cnt = cnt +1
	
	propA:moveRot ( 360, 360, 69, 5 ) 
	
	local action = propA:moveLoc (cordinates[cnt][1], cordinates[cnt][2],cordinates[cnt][3], 6, MOAIEaseType.LINEAR )	
	action:setListener ( MOAIAction.EVENT_STOP, actionDone )

	if cnt == #cordinates then
	   cnt = 0
	end
end

local action = propA:moveLoc (cordinates[cnt][1], cordinates[cnt][2],cordinates[cnt][3], 6, MOAIEaseType.LINEAR )	
action:setListener ( MOAIAction.EVENT_STOP, actionDone )
                        
local   myCamera = MOAITransform2D.new ()                                                   
myCamera:setAttrLink ( MOAIProp2D.INHERIT_LOC, propA, MOAIProp2D.TRANSFORM_TRAIT )			
camera:setAttrLink ( MOAIProp2D.INHERIT_LOC, myCamera, MOAIProp2D.TRANSFORM_TRAIT )	      

local function timerfps()   
	local fps = MOAISim.getPerformance()
	  
	if fps > 40 then
		print('','CULLED> FPS',MOAISim.getPerformance(),'LOC',propA:getLoc())
	else
		print('','> FPS',MOAISim.getPerformance(),'LOC',propA:getLoc())
	end 
end   
  
local	timer = MOAITimer.new()
timer:setSpan(0.5)
timer:setMode(MOAITimer.LOOP)
timer:setListener(MOAITimer.EVENT_TIMER_END_SPAN,timerfps,0)
timer:start()
