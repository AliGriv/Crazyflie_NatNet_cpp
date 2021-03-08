file(REMOVE_RECURSE
  "libSensor.pdb"
  "libSensor.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Sensor.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
