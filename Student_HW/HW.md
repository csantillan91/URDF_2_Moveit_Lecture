# Homework: Pick and place in simulation

In Part 1 you turned a URDF into a MoveIt configuration. In Part 2 you used
it to compare planners around obstacles. This homework puts the arm in a
physics simulator and asks you to make it pick up a block, carry it over a
wall, and set it down. The arm is the same; everything you built still
applies. What is new is that the world now pushes back: the block has weight,
the arm takes time to get where it was told to go, and a plan that looks fine
in RViz can still fail on the table.

Most of the work is **choosing and recording waypoints**. The sequencing code
is given to you; the robot poses that make it work are not.

## Survey 
Before you begin, please take 5 minutes to fill out this survey. As I mentioned in class, this is my first time doing this, so I would greatly appreciate your feedback. 

https://forms.gle/kEKmvEvknGYUnuJN6

## What you are given

The repository contains finished versions of Parts 1 and 2 (`README.md`,
`PART2.md`, `buzz_arm_description`, `buzz_arm_moveit_config`,
`buzz_arm_planning_lab`) plus two new packages:

- **`buzz_arm_sim`**: the Gazebo world (a table top and a 4 x 4 x 15 cm block),
  the wall in both of its forms (Gazebo model and planning-scene box), a
  second copy of the robot description for Gazebo, and launch files. Read `urdf/buzz_arm_sim.urdf.xacro` next to Part 1's
  `buzz_arm_moveit_config/config/buzz_arm.urdf.xacro`: the ros2_control block
  is identical except for one line, the `<hardware><plugin>` that names Gazebo
  instead of the mock hardware. MoveIt, the controllers and their YAML files
  are untouched. That is the point of ros2_control.
- **`buzz_arm_hw`**: five small scripts, all `ros2 run buzz_arm_hw <name>`:

  | script | what it does |
  |---|---|
  | `pick_place` | the sequencer: the steps are written, the poses are yours |
  | `check_sim` | preflight: is exactly one of everything running? |
  | `reset_block` | puts the block back on its spot |
  | `ik_check` | which way can the gripper point from a given spot (experiment A) |
  | `cartesian_check` | how much of a straight line can the gripper follow (experiment B) |

  Every script has a docstring at the top that explains what it talks to and
  why. Read them; they are short.

**About the grasp.** Gazebo simulates contact, but a two-finger gripper holding
a block by friction is unreliable in any simulator and is not what this course
is about. So grasping is a command: publishing on `/buzz_arm/attach` welds the
block to the palm link (`Link_5`) *wherever the block is at that instant*, and
`/buzz_arm/detach` releases it to gravity. The fingers still move and still
collide. Two consequences you should keep in mind: if the gripper is not at the
block when you attach, the block is welded at whatever offset it had and comes
along at an angle; and MoveIt knows nothing about the weld, so you also have to
tell the planner it is carrying something (section 4).

## 0. Before you start

1. Packages, on top of the Part 1 list:

   ```bash
   sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
   ```

2. If your session is Wayland (`echo $XDG_SESSION_TYPE`), add this to
   `~/.bashrc` next to the two lines from Part 1, or RViz and Gazebo's window
   will crash on startup:

   ```bash
   export QT_QPA_PLATFORM=xcb
   ```

3. Build from a clean tree. If you copied this repository from somewhere
   else, delete `build/`, `install/` and `log/` first; they contain absolute
   paths from the machine they were built on.

   ```bash
   cd buzz_robot_arm
   rm -rf build install log
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Launch, and run the preflight:

   ```bash
   ros2 launch buzz_arm_sim sim_moveit.launch.py
   ```

   Two windows: Gazebo (arm on a brown table, orange block to its front-left)
   and RViz (the MotionPlanning panel from Part 1). In a second terminal:

   ```bash
   ros2 run buzz_arm_hw check_sim
   ```

   Every line must say `OK`. The two that matter most are "1 server(s)
   publishing" and "controller_manager nodes: 1". If either says 2, a Gazebo
   from an earlier launch is still running: your controllers are in one world
   and the window shows the other, and nothing below will behave. See
   Troubleshooting. Run `check_sim` every time you relaunch; Ctrl-C does not
   reliably stop Gazebo.

## 1. Meet the simulation

With the launch running, in the second terminal:

```bash
ros2 control list_controllers            # three, all active
ros2 topic echo /joint_states --once     # seven joints; the mimic finger is one of them now
ros2 topic list | grep buzz_arm          # /buzz_arm/attach, /buzz_arm/detach, /buzz_arm/attached
```

Plan and execute something from RViz exactly as in Part 1: goal state
`reach_low`, Plan & Execute. Watch the Gazebo window, not RViz. Then look at
the move_group terminal: after `Completed trajectory execution with status
SUCCEEDED`, run `ros2 topic echo /joint_states --once` immediately and compare
`Revolute_2` with 2.6. It is a few hundredths off and still moving: the
simulated arm is being *driven* to the goal, not teleported like the mock
hardware was. Remember this for section 3.

Now the weld, by hand, with the arm nowhere near the block:

```bash
ros2 topic pub --once /buzz_arm/attach std_msgs/msg/Empty "{}"
```

Plan & Execute to `home`. The block flies across the table with the arm. That
is what a grasp issued at the wrong moment looks like. Release it:

```bash
ros2 topic pub --once /buzz_arm/detach std_msgs/msg/Empty "{}"
```

It falls. Put it back for the next section:

```bash
ros2 run buzz_arm_hw reset_block
```

## 2. Your waypoints (the main part)

The sequence needs five arm poses and one gripper opening. The names are
fixed, because `pick_place` looks them up:

| name | group | what it is |
|---|---|---|
| `pick_hover` | `buzz_arm` | above and in front of the block, gripper open, fingers pointing at it from the side |
| `pick_grasp` | `buzz_arm` | fingers around the block |
| `place_hover` | `buzz_arm` | above the place spot, on the far side of the wall |
| `place_grasp` | `buzz_arm` | block resting on the table at the place spot |
| `retreat` | `buzz_arm` | up and away from `place_grasp` so the open fingers clear the block |
| `grab` | `gripper` | `Revolute_RFinger` closed enough to touch the block, not crush it |

`home`, `open` and `closed` exist from Part 1. The block starts at
(0.39, 0.15) in `base_link`, standing on the table, 15 cm tall. The wall
(section 4) is a 4 cm thick slab running outward from the arm's base at
60 degrees azimuth; the block is at 30 degrees. The place spot is yours to
choose: anywhere past the wall, towards +y, that the arm can reach. Where you
put it is not graded; how you got there is.

**How to find a pose.** Three tools, use all three:

1. **RViz, Joints tab** of the MotionPlanning panel: sliders for the five arm
   joints. Drag until the goal-state ghost looks right, Plan & Execute, watch
   the Gazebo window. This is slow and exact.
2. **RViz, interactive marker** at `tool0`: drag the gripper where you want it
   and let IK find the joints. Fast when it works. It will refuse most
   orientations you ask for; you saw in lecture why a five-joint arm cannot
   take an arbitrary six-number pose, and experiment A measures it.
3. **`tf2_echo`** to record where you actually are:

   ```bash
   ros2 run tf2_ros tf2_echo base_link tool0
   ```

   Translation is tool0's position in metres. Save the full output for your
   `pick_grasp` pose, including the 4 x 4 matrix; the write-up asks about it.

When a pose is right, read the joint values from `/joint_states` and add a
`<group_state>` to the `buzz_arm.srdf`, as you did for `reach_low` in Part 2. Rules:

- Every value strictly inside the URDF limits. `Revolute_3` in particular
  likes to end up at 3.1 when the fingers reach down; the limit is 3.14159.
- Approach the block **from the side**, fingers horizontal, pointing away from
  the base.
- The weld keeps the block at a fixed offset from the gripper. If `pick_grasp`
  holds the block with tool0 at some height above the table, `place_grasp`
  needs tool0 at that **same height**, or you will set the block down inside
  the table (section 4 shows you what the planner thinks of that).
- Relaunch after editing the SRDF. move_group reads it once.

Check each pose with `plan_benchmark --goal <name> --execute` (it waits for
the arm to settle before planning, and prints *why* a plan failed). Then do the
grasp by hand once: `pick_hover`, `pick_grasp`, close the gripper to your
`grab` value, `/buzz_arm/attach`, `place_hover`. If the block hangs straight,
your poses are good. If it hangs at an angle, the gripper was not where you
thought it was when you attached.

Gripper command for the manual test:

```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [Revolute_RFinger], points: [{positions: [0.45], time_from_start: {sec: 1}}]}}"
```

## 3. Run the sequence

Open `src/buzz_arm_hw/buzz_arm_hw/pick_place.py`. It is the same shape as
`plan_benchmark.py`, and in fact imports from it: one `MoveGroup` action goal
per step, named poses looked up in the SRDF, plus two functions that publish
attach and detach. The step list is at the top. Run it:

```bash
ros2 run buzz_arm_hw pick_place
```

Expected: home, open, hover, descend, close, attach, lift, carry, lower,
detach, open, retreat, home. The block ends up standing at the place spot.

After every edit to a script in `buzz_arm_hw`, rebuild before running it:
`colcon build --packages-select buzz_arm_hw`. The copy that `ros2 run` executes
lives in `install/`, and `--symlink-install` does not link Python packages on
this setup.

Two things are deliberately missing from the skeleton, and the first run will
show you both:

- **It does not wait.** Each command goes out the moment the previous action
  returns. From section 1 you know the arm is still settling then. Some of the
  arm moves will come back `CONTROL_FAILED`, and the move_group terminal says
  why: `start point deviates from current robot state more than 0.01 at joint
  'Revolute_3'`. The planner planned from where the arm was supposed to be;
  the controller refused because the arm was not there yet. Fix it by waiting
  until the `/joint_states` velocities are near zero before each step. The
  function already exists, `wait_until_still()` in `plan_benchmark.py`, and
  `PickPlace` inherits it. Do not fix it with a fixed sleep.
- **It does not check.** `run_step()` returns the `MoveItErrorCodes` value of
  each move and the main loop ignores it, so the sequence happily detaches a
  block it never picked up. Make it stop at the first failed step and print
  which step and which error name. A sequence that "finishes" after silently
  skipping the grasp is worse than one that stops.

## 4. The wall

Relaunch with the wall:

```bash
ros2 launch buzz_arm_sim sim_moveit.launch.py wall:=true
# second terminal
ros2 run buzz_arm_hw check_sim
```

The wall now exists in two places, and it has to: in Gazebo (physical, the
block can hit it) and in MoveIt's planning scene (the planner avoids it). A
wall the planner does not know about is one the arm drives into. Look for the
grey slab in the Gazebo window and the same slab in RViz; if RViz does not
show it, the launch's `scene_objects` step did not run, and
`ros2 run buzz_arm_planning_lab scene_objects load wall --file $(ros2 pkg prefix buzz_arm_sim)/share/buzz_arm_sim/config/wall_scene.yaml`
loads it by hand. Open both descriptions, `buzz_arm_sim/models/wall.sdf`
with its pose in `sim.launch.py`, and `buzz_arm_sim/config/wall_scene.yaml`,
and convince yourself they describe the same box.

Run your fixed sequence. Watch the carry from `pick_hover` to `place_hover` in
the Gazebo window, and write down the path length the terminal prints for that
step. The planner routes the **arm** around the wall. Does the **block** clear
it? With some poses it clips the wall, with others it happens to pass over;
either way the planner did not know it was there, because the block is not
part of the robot it plans for.

Tell it. `pick_place.py` already contains `planner_attach()` and
`planner_detach()`: read them. One sends an `AttachedCollisionObject` to
`/apply_planning_scene`, a box of the block's size riding on `tool0` with the
finger links as `touch_links`; the other sends the matching detach. Call them
from the `attach` and `detach` steps, next to the Gazebo weld. In RViz the
block appears in purple while it is attached. Run again, reset the block
first:

- The carry path is different, and its length is different. Write it down.
- If your `place_grasp` sets the block down inside the table, the planner now
  refuses it with `GOAL_STATE_INVALID` (move_group: `Unable to sample any valid
  states for goal tree`). Before, it did not know there was a block to push
  into the table. Raise the pose, following the rule in section 2.

## 5. Two experiments

These are the robotics in this homework. Nothing moves in either one. For
each: run it, read the table, explain it in your own words.

**A. Which way can the gripper point?**

```bash
ros2 run buzz_arm_hw ik_check pick_grasp
ros2 run buzz_arm_hw ik_check place_grasp
```

The script takes tool0's position at your pose and asks the IK solver for the
same position with the gripper pointing in nine directions: away from the
base, tilted up and down, straight down, straight up, towards the base,
sideways. For each direction it tries eight rolls of the gripper about its
own axis and reports whether *any* of them has a solution. Which directions
work? The arm has five joints; a pose in space has six numbers. Which number
did you give up, and what does that mean for a gripper that wants to come
down on the block from above?

**B. Straight lines are hard.**

```bash
ros2 run buzz_arm_hw cartesian_check pick_hover pick_grasp
ros2 run buzz_arm_hw cartesian_check pick_hover place_hover
```

Each run asks MoveIt two questions about the same pair of poses. First, for a
straight-line (Cartesian) path of tool0 from A to B, holding B's orientation
all the way: it walks the line in half-centimetre steps, solves IK at each
one, and reports the fraction it managed before IK failed or the arm would
collide. Second, for the ordinary joint-space plan between the same poses: it
runs tool0 through forward kinematics at every waypoint and reports how far
it strayed from the straight line. Why does the joint-space planner always
find a path between two reachable poses while the Cartesian one may not?
Connect your answer to experiment A.

## 6. Write-up

Hand in: your SRDF `<group_state>` entries, your `pick_place.py`, a screenshot
of the Gazebo window mid-carry with the wall, the two carry path lengths from
section 4, and short answers to:

1. Paste the `tf2_echo base_link tool0` output for your `pick_grasp` pose.
   Is what `tf2_echo` computed forward or inverse kinematics, and what was its
   input? In the 4 x 4 matrix, mark which entries are the rotation and which
   are the translation. What does each part tell you about `tool0`: where is
   it, and which way is the gripper pointing? Check the pointing direction
   against what you see in Gazebo.
2. Experiment A: which degree of freedom is missing, and what would you add to
   the arm to get it?
3. Experiment B: joint-space vs Cartesian planning, in two sentences each, and
   when you would want each one.
4. Section 4: before you called `planner_attach()`, what exactly did the
   planner not know, where in the system did you fix that, and why is the
   planner's copy of the block separate from Gazebo's?
5. Section 3: the sequence that does not wait. What is the difference between
   the mock hardware of Part 1 and the simulated arm that makes this matter,
   and why would it matter even more on the real servos?
6. One thing that went wrong and how you found out what it was.

## Troubleshooting

- **`check_sim` reports 2 servers or 2 `controller_manager` nodes; or MoveIt
  says success and the arm in Gazebo does not move.** Two Gazebos are running.
  Close every terminal that launched anything, then:

  ```bash
  pkill -9 -f "gz sim"; ros2 daemon stop
  gz topic -l | grep -c "^/world/"     # must print 0 before you relaunch
  ```

- **`CONTROL_FAILED`, and move_group says `start point deviates from current
  robot state more than 0.01`.** The arm is still moving from the last
  command. Wait, or fix the sequence (section 3).
- **`START_STATE_IN_COLLISION` right after attaching.** The planner thinks
  the attached block is touching something, usually the table or the wall.
  `reset_block`, and look at where your `pick_grasp` puts the block.
- **The block is stuck to the palm at a strange angle.** You attached when the
  gripper was not at the block. `reset_block`, and check the pose you attach
  from.
- **`GOAL_STATE_INVALID` / `Unable to sample any valid states for goal
  tree`.** Same as Part 2 question 5: the goal pose itself is in collision,
  here usually fingers in the table or in the block, or, after section 4, the
  attached block in the table. Raise the pose a centimetre.
- **`No group_state 'pick_hover' for group 'buzz_arm'`.** `pick_place` found
  your SRDF but not that pose. Check the name and the group, and that you
  relaunched after editing.
- **Gazebo window is black or very slow.** The window is optional; the physics
  does not need it. `ros2 launch buzz_arm_sim sim_moveit.launch.py gui:=false`
  and watch the arm in RViz instead. Everything in this homework can be read
  from RViz plus `gz model -m block --pose`.
- **Edited the URDF or a `buzz_arm_hw` script, nothing changed.** Unlike the
  YAML files, those are copied into `install/`, not linked. Rebuild the
  package you edited.

## On using AI tools

Same rule as Parts 1 and 2: say which tools you used and for what, and include
one concrete thing an assistant told you that was wrong or did not match what
you saw. Questions 1, 4 and 6 are about your own numbers and your own screen.
