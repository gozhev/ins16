### Compilation & Installation:
see ../README.md

### Printing help:
```
$ ./nav -h
```

### Running in debug mode:
Run on board:
```
$ ./nav
```
This prints raw data from sensor to stdout.

### Collecting samples:
Run on board:
```
$ ./nav -f test001 -s 30
```
This put 30 seconds of samples to test001 in binary format.

### Calculating trajectory:
Run on host:
```
$ ./nav -f test001 -a 10
```
This write trajectory coordinates to out.dat file in text format.  
First 10 seconds of samples are used for calibration.

### Plotting:
I recommend to use gnuplot for plotting.
To plot three-dimensional curve run:
```
$ gnuplot plot.p -
```
You can select zoom area by clicking right mouse button.
You can rotate plot using left mouse button.

