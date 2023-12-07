# cybathlon_eog

## Input
The input is taken from the topic 
```/neurodata_filtered```. Therefore, we assume that the data are already filtered. 


## Output
Publish an event in the ```/event/bus``` with code 1024 when an EoG is detected, then publish an event 1024+0x8000 when the timing for the EOG detection is expired (default value: 2sec).

## Rqt dynamic reconfigure
With the reconfigure is possible to change two values used in this package:
- time: which is the time to wait after an EOG detection
- threshold: which is the threshold to overpass in order to detect an EOG

## Cybathlon case
we add a filter chain composed of a band-pass filter between 1 and 10. The file for the filterchain is provided in the folder ```cybathlon_eog/cfg/bandPassCybathlon.yaml```. Addition to our launch file, we need to add the following:
<!--  filter chain  -->
	<rosparam command="load" file="$(find rosneuro_filters_butterworth)/cfg/bandPassCybathlon.yaml"/>
	<node name="filterchain_node" pkg="rosneuro_filters" type="filterchain_node" output="screen" >
		<param name="configname" value="ButterworthBandPass" />
	</node>	
