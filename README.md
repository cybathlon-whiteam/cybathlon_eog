# cybathlon_eog

## input
The input is taken from the topic 
```/neurodata_filtered```. Therefore, we assume that the data are already filtered. 


## output
Publish an event in the ```/event/bus``` with code 1024 if a EoG is detected.

## cybathlon case
we add a filter chain composed by a band-pass filter between 1 and 10. The file for the filterchain is provided in the folder ```cybathlon_eog/cfg/bandPassCybathlon.yaml```. Additonally to our launch file we need to add:
<code>
<!--  filter chain  -->
	<rosparam command="load" file="$(find rosneuro_filters_butterworth)/cfg/bandPassCybathlon.yaml"/>
	<node name="filterchain_node" pkg="rosneuro_filters" type="filterchain_node" output="screen" >
		<param name="configname" value="ButterworthBandPass" />
	</node>	
</code>