all:
	g++ karaaraci.cpp ../src/JHPWMPCA9685.cpp -I../src -li2c -o karaaraci

