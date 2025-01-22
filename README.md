# <ins>Unity ML-Agents Project for Agent Control</ins>

## Project Description
- The purpose of this project is to train and evaluate autonomous agents in the Unity game engine using the ML-Agents toolkit. By applying Deep Reinforcement Learning (DRL) algorithms, we aim to teach these agents to make decisions and perform tasks effectively in a 3D environment. Specifically, the project involves setting up training environments, adjusting learning parameters, and implementing custom sensor inputs to optimize the agents' performance. The overall goal is to analyze how well different DRL algorithms and environmental setups contribute to agent learning and behavior in complex, dynamic scenarios.

## Objectives
- Set up the necessary tools: Install and configure Unity, the ML-Agents toolkit, and Git for version control to ensure a smooth development process.
- Create a project plan: Develop a detailed plan for the entire semester, outlining key milestones, tasks, and timelines to guide the project's progress.
- Familiarize the team with tools: Ensure that all team members understand how to use Unity, ML-Agents, and Git, covering both basic and advanced functionalities.
- Make small code modifications: Implement minor changes to the ML-Agents environment to gain a hands-on understanding of how agents, algorithms, and environments interact. This experimentation will lay the groundwork for more complex tasks in the future.

## Installation and How to Run
1. **Install Unity**: [Download Unity](https://unity.com/)
2. **Install GIT**: [Download GIT](https://git-scm.com/downloads)
3. **Python and Virtual Environment**: Follow the [steps](https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Installation.md) to set up Python and create a virtual environment for ML-Agents.
4. **Git-Hub Repository Cloning**: Clone the repository from [recommended branch](https://github.com/AIML21/main.git) using ```git clone``` command in your command prompt or extracting the compressed file.
5. **Unity Setup With The Git**: Open the project file in Unity which is located in ```Main\ml-agents\Project``` of the git repository after pulling the ```main``` branch and create add a new project to unity.
6. **Follow the Unity Setup Process**: This will conclude the setup.
7. **Navigating the Program**: Once launched, in the unity editors folder tab navigate to ```\Assets\ML-Agents\Examples\Soccer\Scenes``` and double click the ```SoccerTwos``` scene to open the scene.
8. **Playing the Simulation**: When the process is complete, there are multiple ways to launch the simulation.
   - You may press the play button the top of the unity game screen.
   - You may build the project to access the game through a executable. Follow the [tutorial](https://docs.unity3d.com/550/Documentation/Manual/PublishingBuilds.html) for this.
   - You may run the executable or the training via the command promp. However this step is unique for each user and requires a different code, learn more [here](https://github.com/Unity-Technologies/ml-agents/blob/release_19_docs/docs/Learning-Environment-Executable.md). The overall structure for the training command is as follows.
   ``` "Path to where your python version 10 or 11 is installed with ml-agents installation\Python\Python310\Scripts\mlagents-learn" "/Main/ml-agents/config/poca/SoccerTwos.yaml" --run-id=NameOfYourTraining --train ```

## Team Members
- Team Members Name - (Their Github Nickname)
1. Julia - (julca2706)
    - Addition of particles for Worm Agent
    - Member of sound team.
2. Kennedy - (Kennedy531)
    - Changed team colors for football agent
    - Member of memory team.
3. Mika - (mikahagenbeek692)
    - Increase Sorter Agent speed
    - Member of vision team.
4. Mila - (MilaSpasova)
    - Adding color-changing logic to Crawler Agent
    - Member of sound team.
5. Levent - (Itzy-B)
    - README file, In Wall Jump Agent, changed, speed of movement, wall height range, jumping time, the obstacle height, the field size
    - Member of vision team.
6. Florien - (front-depiction)
    - Implement dynamic movement sensitivity for Ball3D Agent
    - Member of memory team.
7. Srijith - (Glitch719)
    - Altering walker movement speed
    - Member of memory team.

## Branches and Purposes
The following will describe the branches and what they are used for. In order to update the project you are required to pull the branches from github seperately, using the ```git pull```, based on the version you want.
1. ```main```:
   - This branch the final one that we have decided to go with. It contains the implemented vision angle input to the agents along side a better trained model with optimized parameters. This branch also includes a custom UI showing the statistics of the current running simulation. This feature can be hidden using the ```Enter``` key.
2. ```FixedMemorySystem```:
   - This is the final branch that the momory team worked with. The memory team developed a system to improve agents' decision-making by utilizing recent visual data in a FIFO memory structure. The data, normalized for consistency, was initially stored in a 30-frame buffer containing metrics like distance to the goal and ball position. Despite attempts to reduce computational load by shortening the buffer to 10 and 3 frames, training issues persisted.
3. ```SoundTeam```:
   - This branch is the final one that uses sound as an input. There have been multiple modifications, however the main idea is to have a sphere that is the sound zone and detect the ball as the agents get in range. The reward system has been altered in this branch to use a more optimal and precise one and to prevent self-goals. These changes have been tested with newly trained models to find the most optimal outcome.
4. ```RayCast_With_JumpExperiment```
   - This is an alternative branch that works with jumping function that makes the agents jump every 5 seconds when they detect an opposing agent withing their vicinity. This branch is anm experiment and hasn't been trained.

## Current Status
- The current state of the project is that it has been seperated into modules with different inputs and agent changes. It runs perfectly as required and all tools are installed. The agents on multiple branches are trained and training files are in the ```\Main\ml-agents\config\POCA``` file. This is the final module however it can include all sorts of changes to optimize the results or alter the modules purpose.

## Contribution Guidelines
We welcome contributions to this project! Please follow these steps to contribute:

1. <ins>**Creating Branches For Editors**:</ins>
- Clone the repository:
    - ```git clone https://github.com/AIML21/main.git```
    - ```cd main```
- Before creating a new branch, ensure your local repository is up to date:
    - ```git pull origin main```
- Create a new branch for your work:
    - ```git checkout -b <branch-name>```

Note: Use clear, descriptive names for branches, such as feature-add-custom-sensor or bugfix-training-algorithm.

 <br>
 
2. <ins>**Altering The Project For Users**:</ins>
- Clone the repository:
    - ```git clone https://github.com/AIML21/main.git```
    - ```cd main```
- Delete the git file in the downloaded file location:
    - Navigate to the location where you have cloned our repository.
    - Find and delete ```.git``` folder (note that this is a hidden folder, you might need to enable hidden folders on your file explorer).
- Add folder to your own repository and push:
    - Use the following commands on your command prompt ```git add -a```, ```git commit -m "Added The ML-Agents Project"``` and ```git push``` to push to your own repository.
- Now you may alter the project all on your own repository. Please do not forget to mentions us and creators in the aknowledgment section.

<br>

3. <ins>**Making Commits**:</ins>
- After making changes, add the files you want to commit:
    - ```git add <file-name>``` or ```git add -a```.
- Commit your changes with a meaningful message:
    - ```git commit -m "Describe your changes"```
- Push your branch to the remote repository:
    - ```git push origin <branch-name>```


## License
MIT License

Copyright (c) 2025 [Unity ML-Agents Project for Agent Control]

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Acknowledgments
We would like to thank the following resources and individuals for their support:
    - Unity ML-Agents Toolkit: ML-Agents GitHub Repository
    - Documentation: Unity and ML-Agents official documentation Unity Documentation and ML-Agents Toolkit Documentation
    - Special thanks to every team member and [miguelalonsojr](https://github.com/miguelalonsojr) for their contributions to the project.
