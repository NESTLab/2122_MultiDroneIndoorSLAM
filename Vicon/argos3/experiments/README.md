# Experiements

These expermiments demonstrate the various abilities of this plugin. The experiments denoted __Physical__ are ment to run in conjunction with the Vicon motion capture system and the physical robots (usually the Khepera IVs). Experiments denoted by __Simulation__ can be run entirely in the argos simulator.

In many cases there are simulator and Vicon versions of the experiements included. In these cases the experiment ending in simulator.argos is ment for running entirely in argos while those with the vicon.argos suffix use the vicon Motion Tracker and the physical Khepera IV robots. 

## Utility Experiments

### battery\_vicon Experiment (Physical)

This demonstrates the ability to retreive the battery status from the real Khepera IV robots. The battery statuses of the robots are printed to the Argos logger.

### controller\_allocator\_test Experiment (Simulator)

This is a simeple experiment to show the ability to assign controllers to robots dynamically as. The experiment uses the spam updater to create 10 Khepera IV robots. The controller assigner for the Kheperas to the controllers as noted below.

Khepera ID | Controller Name
--- | ---
Khepera_1 | diffusion1
Khepera_2 | diffusion1
Khepera_3 | diffusion2
Khepera_4 | diffusion2
Khepera_5 | diffusion2
Khepera_6 | diffusion2
Khepera_7 | diffusion3
Khepera_8 | diffusion3
Khepera_9 | diffusion1
Khepera_10 | diffusion1

### csv\_updater\_test Experiment (Simulator)

This experiment shows receiving updated from a CSV file. 3 Kilobots read data from the test.csv file in the root directory of this project. The positions in this file were originally generated randomly so the robots with jump around when this experiment is run.

## kilotrack Experiment (Physical)

This experiment works inconjuction with the Kilotrack software and the kilobot table in the NEST lab, it displays the locations of the Kilobots marked with the correct aruco tags.

## Virtual Stimergy Experiments 

These experiements are all run to test the disperal and persistance of virtual stimergy table entries. All of these experiemnts are run in the simulator.

## Larger Experiments

These experiements show the ability to take an experiement written to use in the Argos simulator using either argos or buzz controllers and adapt them to be used with the Vicon system. In all of these experiements, the physical version uses the st_combined proximity sensor so that physical and simulated obstical can be avoided.

### Diffusion (Simulator/Physical) 

The robots atempt to diffuse into the given space using a simple diffusion and obstacle aviodence system.

### Pheromone (Simulator/Physical) 

This experiments shows a simple behaviour using a medium to represent pheromone trails. One robot is designated as the layer and moves around the space with a diffusion behavior, laying a trail of pheramones. The other robots are followers who also diffuse in the space, but once they detect some pheromone they will attempt to follow the trail.

### Persistant Coverage (Simulator/Physical) 

In this experiement, the robots attempt to activate points of interest (POI) so that the time between activations is minimized. The robots also have a concept of battery life and must periodically go to a recharging location.
