file(REMOVE_RECURSE
  "libTrajectory_Planner.pdb"
  "libTrajectory_Planner.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Trajectory_Planner.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
