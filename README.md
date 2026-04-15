# rosneuro_integrator_buffer

## Params
- `n_classes`: used to preallocate prediction buffers. Set it to the number of classes coming from `/neuroprediction` messages.
- `buffer_size`: its inverse defines the base step increment at each frame. Used to control the minimum number of frames needed to fill a buffer.
- `thresholds_rejection`: passing a vector of probabilities `[n_classes]`, it establishes the minimum required probability for the highest class to trigger an integration step. If the probability is below this threshold, the buffer holds its value.
- `k_gain`: float multiplier used exclusively in `SOFT` integration mode to dynamically scale the integration velocity.
- `increment`: determines the strategy used to update the buffers based on the probabilities for the class $c$:
    - **0 HARD (default)**: determines the highest scoring class $c$ from the probabilities and computes a fixed step:
        
        $\quad b_i^{(t+1)} = \max(0, \min(1, b_i^{(t)} + 1/\text{buffer\_size})) \quad \text{if } i=c$  
        $\quad b_i^{(t+1)} = \max(0, \min(1, b_i^{(t)} - 1/\text{buffer\_size})) \quad \text{if } i\ne c$  
    
    - **1 SOFT**: determines the highest scoring class $c$ and its probability $P(c)$, then computes a dynamic step bounded by `k_gain`:
        
        $\quad \text{velocity\_factor} = \min(1.0, |P(c) - 0.5| \times 2.0 \times \text{k\_gain})$
        $\quad \text{step} = (1 / \text{buffer\_size}) \times \text{velocity\_factor}$
         
        $\quad b_i^{(t+1)} = \max(0, \min(1, b_i^{(t)} + \text{step})) \quad \text{if } i=c$
        $\quad b_i^{(t+1)} = \max(0, \min(1, b_i^{(t)} - \text{step})) \quad \text{if } i\ne c$
- `init_val`: sets the initial value for each buffer after a reset.

## Example usage
```xml
<?xml version="1.0"?>
<launch>
	<arg name="plugin" default='rosneuro::integrator::Buffer'/>
	<arg name="n_classes" default="2" /> 
	<arg name="buffer_size" default="256" /> 
	<arg name="increment" default="1" /> 
	<arg name="init_val" default="[0.5, 0.5]" /> 
	<arg name="k_gain" default="1.0" /> 
	<arg name="thresholds_rejection" default="[0.55, 0.55]" /> 

	<node name="integrator" pkg="rosneuro_integrator" type="integrator" output="screen" >
		<param name="plugin" value="$(arg plugin)"/>
		<param name="n_classes" value="$(arg n_classes)"/>
		<param name="buffer_size" value="$(arg buffer_size)"/>
		<param name="increment" value="$(arg increment)"/>
		<param name="k_gain" value="$(arg k_gain)"/>
		<rosparam param="init_val" subst_value="True">$(arg init_val)</rosparam>
		<rosparam param="thresholds_rejection" subst_value="True">$(arg thresholds_rejection)</rosparam>
	</node>
</launch>
```