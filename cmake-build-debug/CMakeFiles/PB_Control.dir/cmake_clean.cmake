file(REMOVE_RECURSE
  "libPB_Control.pdb"
  "libPB_Control.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/PB_Control.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
