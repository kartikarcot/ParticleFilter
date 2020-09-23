## Installation prereqs
```
sudo apt-get install libjsoncpp-dev libopencv-dev
```

## Commands to run for Mac
```
export CC=gcc
export CXX=g++
cmake -DCMAKE_OSX_SYSROOT="/" ../code/ 
mv compile_commands.json ../code
```
## Add this to .bashrc to make development easier
* Change the path variable
```
function pf() {
	path="/Users/stark/Projects/16833_HW1_ParticleFilter"
	alias dev="cd $path/code"
	alias bldD="cd $path/build && cmake -DDEBUG_MODE=ON $path/code && make"
	alias bldI="cd $path/build && cmake -DDEBUG_MODE=OFF $path/code && make"
	alias run="cd $path/build && ./src/main ../code/data/map/wean.dat ../code/data/log/robotdata1.log"
	export PS1="\e[0;31m[\u@\h \W]\$ \e[m "
}
```

## TODO:
- [ ] Add config file
- [ ] Add types.h for standard types used like Map and Pose2D
- [ ] Tune hyperparameters
- [ ] Add tests in separate directory
```
rm z_vals.txt && touch z_vals.txt && ./src/testing && gnuplot -e "set terminal svg; set style data lines; plot 'z_vals.txt'" > plot.svg
 gnuplot -e "set terminal svg; set style data lines; plot 'laser.txt' using 1 with lines title 'simulated', 'laser.txt' using 2 with lines title 'log' " > laser.svg
```
