# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/_deps/picotool-src"
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/_deps/picotool-build"
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/_deps"
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/picotool/tmp"
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/picotool/src/picotoolBuild-stamp"
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/picotool/src"
  "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/picotool/src/picotoolBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/picotool/src/picotoolBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/zeverlanotebook/Repositories/Kalimba/codes/fixed_main/build/picotool/src/picotoolBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
