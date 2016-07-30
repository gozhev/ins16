set xlabel "x (meters)"
set ylabel "y (meters)"
set zlabel "z (meters)"
set view 60, 30, 2
#set view 0, 0, 2
set view equal xyz
splot "out.dat" u 1:2:3 with lines

