# plot 'sample1.txt' u 1:2 pt 6 lc 'black'
# plot 'sample1.txt' u 1:2:($3 * cos( $4 / 180 * pi) ):($3 * sin( $4 / 180 * pi) ) with vec head filled lc 'black'
plot 'sample1.txt' u 1:2:($3 * cos( $4 / 180 * pi) ):($3 * sin( $4 / 180 * pi) ) with vec head filled lc 'black', '' u 1:2 pt 6 lc 'black'
