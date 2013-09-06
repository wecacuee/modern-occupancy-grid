# configure the GUI window
window
(
  size [ 635.000 666.000 ] # in pixels
  scale 37.481   # pixels per meter
  center [ -0.019  -0.282 ]
  rotate [ 0  0 ]
  			
  show_data 1              # 1=on 0=off
)

# load an environment bitmap
floorplan
( 
  name "cave"
  size [16.000 16.000 0.800]
  pose [0 0 0 0]
  bitmap "bitmaps/cave.png"
)

pioneer2dx
(		  
  # can refer to the robot by this name
  name "r0"
  pose [ -7 -7 0 45 ] # for cave.png

  sicklaser(   
     ctrl "lasernoise"  # uncomment this line to run a laser noise generator
  )
 
  ctrl "wander"

  # report error-free position in world coordinates
  localization "gps"
  localization_origin [ 0 0 0 0 ]
)
