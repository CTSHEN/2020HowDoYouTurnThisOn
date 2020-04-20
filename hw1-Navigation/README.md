###### tags: `RNE Course 2020`
# HW1 - P46974278
This package demos path planning and path tracking for bicycle model.
The Hackmd link is [here.](https://hackmd.io/M2MxtGUSTC-tq8YLD5xCkQ?both)

- Manual
    > $ python main.py [color=#3b75c6]

    A window will pop out,

    :::info 
    Press 'w' 's' 'd' 'a'( acceleration, deceleration, steer left and steer right respectively) to manually control the car. 
    :::
    Choose Control Algoritm and Planning Algoritm
    :::info
    Press 'l' 'k' to choose the Control Algoritm ('l': Pure Pursuit 'k': Stanley)
    Press 'p' 'o'  to choose the Planning Algoritm ('p': Astar 'o': RRT)
    :::
    Set a goal by clicking a point in th map with your mouse. The planner will start to plan following by the car tracking after 1sec. 
    
- Demo 
    [video on google drive](https://https://drive.google.com/drive/folders/1AWsN9_UpGO7M9cbSHntI0e7NPP5wK_mU?usp=sharing)
    1. A* + PP
    2. RRT* + PP
    3. A* + stanley
    4. RRT* + stanley 

- Extra Works
  1. [Bouns] Collison handling: go backwardly and go forward after a while.
![](https://i.imgur.com/CGmsfeu.png)

  2. Trottle Control : generate the path with velocity profile (constant speed here), and calculate the command by kinodynamicl model.
