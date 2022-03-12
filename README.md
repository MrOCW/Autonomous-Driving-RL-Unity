# Autonomous Driving based on Reinforcement Learning in Unity
This is the Final Year Project for my Bachelors in Nanyang Technological University. The car model used is a [Donkey Car](http://www.donkeycar.com), an open source DIY self driving platform for small scale RC cars. However, only imitation learning is officially supported on their platform. This project makes use of Unity's [ML-Agents](https://github.com/MrOCW/ml-agents) package for reinforcement learning, along with [MMSegmentation](https://github.com/open-mmlab/mmsegmentation) for semantic segmentation.

## Features
- **Domain Randomization**
  Lighting, wall colours, and terrain textures are constantly changing during simulation
- **Semantic Segmentation**
  Option to train a policy with the output of a semantic segmentation model. A [SegFormer](https://arxiv.org/abs/2105.15203)-B0 model was trained with synthetic data generated from Unity. 
  Note: Testing in Unity can only be done with ground truth masks as many models, including SegFormer, are not supported by Unity's [Barracuda](https://github.com/Unity-Technologies/barracuda-release) package.
- **Obstacle Avoidance**
  Scene with obstacles in the road for training obstacle avoidance.  

## Setup
Unity 2020.3.11f1  
```
$ git clone https://github.com/MrOCW/Autonomous-Driving-RL-Unity
$ cd Autonomous-Driving-RL-Unity
$ git clone https://github.com/MrOCW/ml-agents ml-agents-2.1-dev
```
Local installation for development  
```
$ cd ml-agents-2.1-dev
$ pip3 install -e ./ml-agents-envs
$ pip3 install -e ./ml-agents
```  
Install ML-Agents in the Unity Editor by following the installation [guide](ml-agents-2.1-dev/docs/Installation.md)

