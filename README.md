# SimFramework

## Overview
SimFramework is a standalone C++ block diagram simulation framework created for my final year masters engineering project: "Eco Academy: An educational video game". The framework was used to create a simulation of an internal combustion engine powered vehicle. This was wrapped into a plugin for the Unreal Engine 4 game engine (plugin repository: https://github.com/tomjhunt13/SimInterface). This was used to communicate between the simulation and the game (game repository: https://github.com/tomjhunt13/EcoAcademy).

Included in this repository is the framework and the vehicle model. Additionally, my dissertation is included for detailed reference.

## SimFramework/Framework
In this directory is the core simulation framework and common block components. 

Simulations consist of blocks connected by signals. Subsystems define groups of connected blocks and can implement additional behaviors. Systems contain signals, blocks and subsystems and provide an interface for the simulation. The file _Framework.h_ contains a template class for signals and base classes representing blocks, subsystems and systems. 

The framework features a topological sorting graph algorithm to determine a suitable block update order. Integration is performed using explicit forth-order Runge-Kutta numerical integration. A zero-order hold assumption is applied to block inputs for simplicity.
