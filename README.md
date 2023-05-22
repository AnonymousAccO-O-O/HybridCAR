# [CARAMEL] CAR:Augmented Method for ExceLLence

## How to make
Nearly all the options are trivially written in Makefile.
For example, to build with no specific optimization, just type:
```
make TARGET=caramel
```
or even just type:
```
make
```

To turn off default options, use command like:
```
make INTER=OFF ROTATE=OFF ... TARGET=caramel-bare
```

To enable diving-graph drawing, use:
```
DIVE=ON
```

To enable hybrid method, which add PreProcessing phases, use:
```
make PP_TIME=XXX MAXNI=XX TARGET=caramel-hybrid
```

## How to run
For example, you can run "./caramel -b -e ./ICCAD/counterp0.aig ./"
For more, see -h for help.

## Acknowledgement
Part of this repo is built up upon simpleCAR. 



