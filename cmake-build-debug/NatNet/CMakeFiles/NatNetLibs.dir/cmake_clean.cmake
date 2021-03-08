file(REMOVE_RECURSE
  "libNatNetLibs.pdb"
  "libNatNetLibs.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/NatNetLibs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
