# configure the GUI window
window
(
  size [ 400.000 300.000 ] # in pixels
  scale 33.33 # pixels per meter
  center [ -0.0  -0.0 ]
  rotate [ 0  0 ]
  			
  show_data 1              # 1=on 0=off
)

# load an environment bitmap
floorplan
( 
  name "simple_rooms"
  size [12.000 9.000 0.800]
  pose [0 0 0 0]
  bitmap "bitmaps/simple_rooms.png"
)

pioneer2dx
(		  
  # can refer to the robot by this name
  name "r0"
  pose [ -2 -0 0 0 ] # for cave.png

  sicklaser(   
     ctrl "lasernoise"  # uncomment this line to run a laser noise generator
  )
 
  #ctrl "wander"

  # report error-free position in world coordinates
  localization "gps"
  localization_origin [ 0 0 0 0 ]
)
