set terminal png
set output 'plot-01.png'

set xrange [ -3.726 : 3.726 ]
set yrange [ -1.200 : 1.200 ]

set size ratio -1

set xlabel 'U-Position [m]'
set xlabel font "Arial,12"

set ylabel 'V-Position [m]'
set ylabel font "Arial,12"

set title 'Planned Path by BFS'
set title font'Arial,12'

plot 'sample1.txt' u 1:2 pt 6 lc 'black' t 'Position', \
     'sample1.txt' u 1:2:($3 * cos( $4 / 180 * pi) ):($3 * sin( $4 / 180 * pi) ) with vec head filled lc 'black' t 'Velocity'

set terminal qt
set output
