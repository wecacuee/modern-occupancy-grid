
# configure the GUI window
window
(
  size [ 3117.000 1189.000 ] # in pixels
  scale 21.87   # pixels per meter
  center [ -0.0    -0.000 ]
  rotate [ 0  0 ]
  			
  show_data 1              # 1=on 0=off
)

# load an environment bitmap
floorplan
( 
  name "hospital"
  size [142.46 54.36 0.800] #in m
  pose [0 0 0 0]
  bitmap "bitmaps/hospital.png"
)

pioneer2dx
(		  
  # can refer to the robot by this name
  name "r0"
  pose [ -6 -5 0 45 ] # for hospital.png

  sicklaser(   
     ctrl "lasernoise"  # uncomment this line to run a laser noise generator
  )
 
  ctrl "wander"

  # report error-free position in world coordinates
  localization "gps"
  localization_origin [ 0 0 0 0 ]
)
