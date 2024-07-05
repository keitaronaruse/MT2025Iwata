set terminal png
set output 'plot-01.png'

set xrange [  0.000 : 7.200 ]
set yrange [ -1.200 : 1.200 ]

set size ratio -1

set xlabel 'u position [m]'
set xlabel font "Arial,12"

set ylabel 'v position [m]'
set ylabel font "Arial,12"

set title 'Planned Path by Dijkstra Search'
set title font'Arial,12'

plot 'sample1.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 1', \
     'sample2.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 2', \
     'sample3.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 3', \
     'sample4.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 4', \
     'sample5.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 5', \
     'sample6.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 6', \
     'sample7.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head t 'Section 7', \
     'sample8.txt' u 1 : 2 : ( $4 * cos($3) ) : ( $4 * sin($3) ) w vec fill head  t 'Section 8'

set terminal qt
set output
