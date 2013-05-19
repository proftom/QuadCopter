These are the Matlab files for image correction. 

the .m file shows a demo of how to correct an immage, the workspace has the necessary variables for it to work. 

Briefly. The variable ReverseStack is the correction matrix. it is a 2 dimentional cell array of vectors, to lookup correction use 
stackReverse{line index}{column index} this will return a table of 56 numbers these are the distances that you should correct to. the input value corresponding to these is held in a 56 long 
vector called reverseDist. 

process:

I want to correct the 3rd pixel allong on the 4th row. 

I read its value from the raw xtion data. lets say it is 1.01m. next I look up the table stackReverse{3}{4}. this gives me the lookup table. 

my value is 1.01m therefor I should lookup which quantisations are closest on the input side using reverseDist. This case it is 1m and 1.1m This can be done using clever indexing rather than checking as the intervals are even.

1m corrisponds to index 6 and 1.1m corrisponds to index 7. 

Lastly I linearly interpolate between stackReverse{3}{4}(6) and stackReverse{3}{4}(7). the interpolation is wieghted by the relitive distances between the input quantizations and the input value, 
