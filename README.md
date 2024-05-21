# rosneuro_integrator_buffer

## Params
- ```n_classes```: used to preallocate prediction buffers. Set it to the number of classes coming from ```/neuroprediction``` messages
- ```buffer_size```: its inverse defines the maximum increment/decrement at each frame. Used to control the minimum number of frames needed to fill a buffer.
- ```increment```: determines the strategy used to update the buffers based on the probabilities for the class $c$:
    - 0 - HARD (default): determines the highest scoring class $c$ from the probabilities and updates the buffers as follow:

        $\quad b_i^{(t+1)} = b_i^{(t)} + 1/\text(buffer\_size)$ if $i=c$  
        $\quad b_i^{(t+1)} = b_i^{(t)} - 1/\text(buffer\_size)$ if $i\neq c$  
    
    - 1 SOFT: determines the highest scoring class $c$ and its probability $P(c)$ and updates the buffers as follow:
         
        $\quad b_i^{(t+1)} = b_i^{(t)} + P(c)/\text(buffer\_size)$ if $i=c$  
        $\quad b_i^{(t+1)} = b_i^{(t)} - P(c)/\text(buffer\_size)$ if $i\neq c$
- ```init_val```: sets the initial value of the buffers after the reset.