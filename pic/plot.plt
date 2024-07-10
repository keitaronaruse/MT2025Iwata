set xrange [ -0.500 : 1.500 ]
set yrange [ -0.500 : 1.500 ]
set zrange [ -0.500 : 1.500 ]
set view equal xyz
set view ,,1

splot 'sample1.txt' u 1:2:3:4:5:6 w vec fill head
pause -1
