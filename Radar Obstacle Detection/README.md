# Radar Target Generation and Detection
## Implementation steps for the 2D CFAR process
 1. Determine the number of Training cells for each dimension. Similarly, pick the number of guard cells. 
 2. Slide the cell under test across the complete matrix. Make sure the CUT has margin for Training and Guard cells from the edges. 
 3. For every iteration, calculate sum of the signal level within the training cells through subtracting sum of the signal level within the guard cells from sum of the signal levels of all the training cells and the guard cells. To sum signal levels, convert the value from logarithmic to linear using db2pow function. 
 4. Average the summed values for all of the training cells used. 
 5. Further multiply the offset to it to determine the threshold. Then, convert the threshold back to logarithmic using pow2db. 
 6. Next, compare the signal under CUT against this threshold. 
 7. If the CUT level > threshold assign the result matrix a value of 1, else leave it with the default value is 0.

## Selection of Training, Guard cells and offset
 Determine the number of Training, Guard cells and offset through increasing their values from the smallest value is 1 until the noise is removed. 

## Steps taken to suppress the non-thresholded cells at the edges
 Create a new zero-matrix for the result of 2D CFAR, just assign 1 to the corresponding cell if it CUT level > threshold. 
 

