# vector-autonomous-delivery-bot
## Description
This was the final lab I did for my CS3630 Class as a part of my CS Minor with a focus on AI & ML. The goal of this project was to write a program for an autonomous robot to follow that directs it to move objects around a warehouse environment while avoiding specific fragile zones. We used a Particle Filter (Monte Carlo) for localization with the help of some marker detection. We used RRT for path planning with the robot. For examples of this project in action please refer to my website. 
## Installation
## Requirements 
- Hardware - Anki Vector Robot 
- Software - Python 3
## Installation Process & Usage
### Scripts Execution
1) *dock.py* to ensure robot is able to pick up object in the arena 
2) *check_battery.py* to ensure robot has enough charge to successfully complete its duties. Anything greater than 3.85 Voltage is good to run. 
3) *main_v2.py* to start autonomous robot's delivery actions 
### Environment Restrictions 
1) Ensure that you use provided markers placed in locations as specified by the objectives pdf as this is what the particle filters are currently configured to. 
2) Set up the mock warehouse environment as specified by the objectives pdf. 
## Credits
- Marc Hoeltge
- CS3630 TA Team

