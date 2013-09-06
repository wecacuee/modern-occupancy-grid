
# configure the GUI window
window
(
  size [ 1086.000 443.000 ] # in pixels
  scale 24.00   # pixels per meter
  center [ -0.0    -0.000 ]
  rotate [ 0  0 ]
  			
  show_data 1              # 1=on 0=off
)

floorplan
( 
  name "hospital_section"
  size [45.25 18.45 0.800] #in m
  pose [0 0 0 0]
  bitmap "bitmaps/hospital_section.png"
)

pioneer2dx
(		  
  # can refer to the robot by this name
  name "r0"
  pose [ -22 3.2 0 00 ] # for hospital_section.png

  sicklaser(   
     ctrl "lasernoise"  # uncomment this line to run a laser noise generator
  )
 
  ctrl "wander"

  # report error-free position in world coordinates
  localization "gps"
  localization_origin [ 0 0 0 0 ]
)
