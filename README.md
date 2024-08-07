#The Compound Double Pendulum
This is simulation of a Compound Double Pendulum. It's C++ under the hood with a python wrapper over it to make it as easy as possible to use.

#Installation and Usage
Download all three files and if you need to, recompile pendulum.cpp (just remember to load the new recompiled library into the python code). To be able to run the code, you need to have python3 installed as well as numpy and pygame. To run the code itself, you need to go to terminal and type "python3 visualize_pendulum.py". If you would like to recompile the code itself, go to the terminal and type "g++ -dynamiclib -o libpendulum.dylib -fPIC pendulum.cpp". 

To run: python3 visualize_pendulum.py
To recompile: g++ -dynamiclib -o libpendulum.dylib -fPIC pendulum.cpp


#Commands
The SPACEBAR key is used to STOP AND START the pendulum, please note that when you use it for simulation. 