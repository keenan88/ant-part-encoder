# ant-part-encoder

_Dependencies:_ 
1. ESP IDF Framework Version 4.3.1
2. Microros for ESP IDF Framework


_Integration into existing project:_
1. Add this repo to your main source code folder and update it's CMake folder to target the include and src files in this repo.
2. Your Cmake file will have include the path to the ESP IDF rotary encoder component in the EXTRA_COMPONENT_DIRS variable, with a line like: set(EXTRA_COMPONENT_DIRS "<$IDF_PATH>/examples/peripherals/pcnt/rotary_encoder/components/rotary_encoder")
3. Your will have to add Micro ros for ESP IDF to the components folder in your repo and add it to your Cmake file.
