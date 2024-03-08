# Imitation Game

This is a simple demo that puts together people detection, ASR, TTS routines and projector capabilities of the robot. Haru does 3 different routines and following them asks the user to replicate. Note there is no monitoring whether user follows Haru or not. Use the `haru_interactions_ws` for this demo.

### Requirements

* haru_unity
* haru_nlp_ros
* [picture_server](https://github.com/rsliu-hri/picture_server.git)

### How to run the demo

Run the simulation environment:

```
roslaunch haru_unity unity_app_launcher.launch
```

Select the projector and Haru scene, ensure it is New Projector.

Run the perception modules and behavior tree, also from the workspace where we have the new BT nodes (catkin_ws for now):

```
roslaunch imitation_game haru_imitation_game_demo.launch
```

Execute the behavior tree:

```
rosservice call /behavior_tree/load_tree "tree_file: '/home/haru/haru_interactions_ws/src/imitation_game/trees/imitating_game.xml'"
```

For a newer version we use the NLP for Yes/No recognition (from haru-nlp package inside haru_interactions_ws). To run with this before executing the BT:

```
cd to haru-nlp (right now, inside catkin_ws)
poetry shell
cd scripts
python3 nlp_yesno_action_server.py
```

This runs script done by Sara that follows the example/script of Yes/No detection that is in the same package.
ROS actions server: /NlpClassifyYesNoFromText
Type of message: haru_nlp_msgs/ClassifyYesNoFromText (this srvs has been added by Sara as well)
It also includes feedback of the kinect camera when the person is imitating the gesture, and the <routine>joy</routine> style tags during speech.

Then execute the tree:

```
rosservice call /behavior_tree/load_tree "tree_file: '/home/haru/haru_interactions_ws/src/imitation_game/trees/imitation_game_v10_yesno_camera.xml'"
```

### Updates of 24th May

Run the demo with the real Haru with the following steps.

```
roslaunch haru_unity unity_app_launcher.launch
```

Select the projector and Haru scene, ensure it is New Projector

In a new terminal:

```
cd to haru-nlp (right now, inside catkin_ws)
poetry shell
cd back to workspace and source devel
cd scripts
python3 nlp_yesno_action_server.py
roslaunch imitation_game haru_imitation_game_demo_real.launch
```

And the following BT:

```
rosservice call /behavior_tree/load_tree "tree_file: '/home/haru/haru_interactions_ws/src/imitation_game/trees/imitation_game_demo4_v4.xml'"
```
