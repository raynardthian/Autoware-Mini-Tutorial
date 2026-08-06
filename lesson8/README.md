[< Previous lesson](../lesson7/) -- [**Main Readme**](../README.md)

# Lesson 8 - Testing in the CARLA simulator

In this final lesson, you will run the whole framework from the previous lessons in closed loop inside the CARLA simulator: the simulated world reacts to your vehicle, and your vehicle must react to the world.

Two tools are used for the closed-loop validation:
* [**CARLA**](https://carla.org/) - an open-source autonomous driving simulator. It renders the world via provided map files (and we will use our own Tartu map), simulates the physics and the sensors (lidar, cameras), and feeds them to your nodes through ROS topics.
* **Visual Scenario Editor (VSE)** - a graphical tool for creating and re-playing driving scenarios in CARLA: NPC vehicles and pedestrians with routes and triggers, traffic light sequences and weather. See the [VSE repository](https://github.com/UT-ADL/visual-scenario-editor) and [how to use the editor](https://github.com/UT-ADL/visual-scenario-editor/blob/main/tutorial.md).

You will first verify that your framework can drive in CARLA, then run it through a prepared VSE scenario, and finally design scenarios yourself where your framework fails.

### Expected outcome
* Understanding how the full autonomous driving stack behaves in a closed-loop simulation
* Exploring the limits of the framework you built


## 1. Run your stack in CARLA

The launch file [lesson8.launch](launch/lesson8.launch) connects your nodes from the previous lessons to CARLA. There is no bag playback: the localization comes from the simulator, and the vehicle commands from your `pure_pursuit_follower` steer the car in the simulation.

By default the detected objects and traffic light statuses come from the simulator's ground truth instead of your perception nodes - simulating the lidar and the cameras is very heavy, and running the perception pipeline on them can slow the simulation down to a crawl. Your planner and controller are still the ones driving. If your machine can afford it, you can enable your own perception with `detector:=cluster` (lesson 5 nodes on the simulated lidar) and/or `tfl_detector:=yolo` (lesson 7 nodes on the simulated cameras).

##### Instructions
1. Start the CARLA simulator:
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -RenderOffScreen
    ```
2. In another terminal, launch your stack:
    ```
    roslaunch autoware_mini_tutorial lesson8.launch
    ```

##### Validation
* RViz opens with the Tartu map and the ego vehicle placed in the simulated city
* The `Carla image view` panel shows the third-person view of the ego vehicle in the simulated world
* Place a goal on the map - the vehicle drives to it


## 2. Run the demo scenario

A driving scenario adds actors to the otherwise empty world: NPC vehicles and pedestrians that spawn, move and react when triggered, and traffic lights that switch according to the scenario triggers. You will run the prepared demo lap scenario and see whether your framework survives traffic.

When your stack is running, VSE automatically detects your ego vehicle and hands the driving over to it - the scenario provides the destination, the other actors and the evaluation.

##### Instructions
1. With `lesson8.launch` running, start VSE and open the `tartu_demo` map. When VSE first launches, it will ask to select the agent's behavior logic. Navigate to `autoware_mini/nodes/platform/carla/` and select `carla_minimal_agent.py`.
2. Open the scenario (`Scenario` menu -> `Open`): `shared/data/scenarios/tartu_demo_route_simplified.json` from the tutorial folder
3. Press **Play**. Note: if your machine has less than 10 Gb VRAM slowdowns are expected.

##### Validation
* The goal appears in RViz automatically and the vehicle starts driving the demo lap
* NPC vehicles and pedestrians act out the scenario around the ego vehicle
* When the run ends, VSE shows a results window scoring the drive (collisions, red light violations, route completion); the same results are also saved as a text file next to the scenario JSON


## 3. Create three failure cases

Your framework from the previous lessons is a simplified one. Remember all limitations that were discussed through the lessons. In this final task you will demonstrate these limits: create three scenarios where your framework fails.

##### Instructions
1. Copy `tartu_demo_route_simplified.json` (e.g. to `failure_case_1.json`) and modify it in VSE - move, add, retime or reroute actors and triggers until your stack demonstrably fails, while a careful human driver would still manage
2. For every failure case, think of a specific change to the framework that would fix it. You do not need implement the fix. The three cases should have three different proposed fixes.
3. Create a `lesson8/scenarios/` folder in your repository and commit the three scenario JSONs there
4. Fill in the three descriptions below: what happens in the scenario, how your framework fails, and what change to the framework would fix it. Add screenshots if needed.
5. Commit and push everything, and be ready to demonstrate your failure cases at the practice session

##### Failure case 1
**Scenario**: while the ego is turning right at an intersection, a pedestrian runs straight across its path and collides with the vehicle.

**What happens**: mid-turn, the ego is moving along a curved section of the local path. A pedestrian who was not yet a concern a moment earlier runs into the road and is almost immediately inside the vehicle's path — there is very little time between the pedestrian becoming relevant and the point of impact, and the vehicle does not manage to stop or swerve away in time.

**Why the framework fails**: the collision checker only reacts to objects whose position *already* overlaps the vehicle's path corridor right now — it has no concept of where a moving pedestrian will be a second or two from now, so a person who is running (rather than walking or standing still) closes the remaining distance far faster than the planner's braking assumptions expect. This is made worse during a turn specifically because nothing in the stack slows the vehicle down for the turn itself (see failure case 3's cause) — so the ego may already be entering the turn faster than it should be, leaving even less margin to react to a sudden pedestrian than it would have going straight. The combination of "no anticipation of fast-moving pedestrians" and "no speed reduction for turns" is what turns a survivable situation into a collision.

**Recommended fix**: give the planner a short-term prediction of where nearby pedestrians and vehicles are heading, based on their current speed and direction, and treat a predicted future overlap with the path as a collision point too — not just an overlap happening right now. This would let the vehicle start slowing down as soon as a fast-moving pedestrian is heading toward its path, instead of waiting until they are already in the way. Combined with slowing the vehicle down through turns in general, this would give it enough distance and time to stop safely.

##### Failure case 2
**Scenario**: a traffic light turns red at the last moment while the ego is approaching the intersection at a moderate speed, and the ego drives straight through it.

**What happens**: the light switches to red only shortly before the ego reaches the stop line, while the vehicle is still travelling at a moderate speed. Instead of slowing down and stopping before the line, the vehicle does not brake at all — it just passes straight through the intersection at its approach speed, as if the light were still green. A careful human driver, seeing the light change ahead of time, would already be easing off the accelerator well before the stop line and would have no trouble stopping in time.

**Why the framework fails**: the planner/controller pair only reacts to a red light once it is already confirmed, and then relies on the stop line as the trigger for braking — there is no notion of building a speed profile in advance that anticipates an upcoming signal. Because the light changes so close to the intersection, the confirmed-red state and the associated stop-line trigger arrive too late relative to the vehicle's limited reaction margin: the planner does not get a fresh, in-time stopping plan generated before the vehicle is already past the point where braking would even matter, so no deceleration command is issued at all and the vehicle simply drives through.

**Recommended fix**: add a traffic-light-aware speed profile that starts shaping the vehicle's speed well before the stop line rather than only reacting once close to it. As the vehicle approaches an intersection with an upcoming or currently red light, the planner should begin decelerating earlier, giving the controller more distance and margin to bring the vehicle to a smooth stop. This earlier, anticipatory deceleration would let the vehicle respect red lights reliably and without last-second, oscillating braking.

##### Failure case 3
**Scenario**: a bus in the lane next to the ego suddenly merges into the ego's lane, and the two vehicles collide.

**What happens**: the bus is initially driving alongside the ego in a neighboring lane, not directly in its path. It then changes lanes into the ego's lane, and by the time it is actually inside the ego's path, there isn't enough distance left for the ego to brake or steer away in time.

**Why the framework fails**: the vehicle only pays attention to its own lane — it checks for obstacles inside a narrow corridor directly around its intended path, and has no awareness at all of vehicles in neighboring lanes that could merge in. A human driver keeps half an eye on vehicles beside them, especially ones that seem to be drifting toward their lane, and would already be easing off the accelerator before the bus fully committed to the lane change. Our framework has no equivalent — a neighboring vehicle simply doesn't exist to the planner until it has already crossed into the ego's own narrow corridor, at which point it may be too late to react.

**Recommended fix**: widen the area the vehicle pays attention to beyond just its own lane — monitor a broader zone that includes neighboring lanes, and treat a vehicle that is drifting or angled toward the ego's lane as an early warning sign, even before it has fully crossed over. This would let the vehicle start slowing or creating space as soon as another vehicle looks like it might merge in, rather than only reacting once the merge is already complete.
