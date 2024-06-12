set view equal xyz
set view ,,2

set xrange [  0.000: 7.452 ]
set yrange [ -1.200: 1.200 ]
set zrange [  0.000: 1.200 ]

splot 'uvw_cont_ramp.txt' u 1:2:3 w lp pt 7 ps 2 lc 8 t 'Continuous Ramp', \
      'uvw_pylons.txt' u 1:2:3 w lp pt 7 ps 3 lc 7 t 'Pylons'
