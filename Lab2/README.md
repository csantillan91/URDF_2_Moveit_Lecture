# Part 1: From a URDF to a MoveIt configuration

You are given the description of a small 5-DOF robot arm with a two-finger
gripper: a URDF and the STL meshes it references, packaged as a ROS 2 package
called `buzz_arm_description`. Nothing else. By the end of this part you will
have turned it into a MoveIt 2 configuration that plans collision-free motions
in simulation, and you will be able to explain every file you generated.

There is no physical robot this semester. Everything runs in simulation using
ros2_control's mock hardware, which the MoveIt Setup Assistant configures for
you.

One rule that will save you hours: **when something fails, the reason is in
the terminal running move_group, not in RViz.** RViz only says "aborted".

## 0. Environment

Ubuntu 24.04 with ROS 2 Jazzy. On a fresh machine:

```bash
# ROS 2 Jazzy itself: follow https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers ros-jazzy-xacro \
                 ros-jazzy-joint-state-publisher-gui \
                 python3-colcon-common-extensions python3-rosdep terminator
sudo rosdep init && rosdep update
```

Put these two lines at the end of `~/.bashrc` so every terminal has them:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

The second line keeps your ROS nodes from discovering everyone else's on the
lab network. Without it, another student's robot can show up in your RViz.

**Terminator.** You will have three or four terminals open at once. Terminator
splits one window into panes. Install it and start it by name:

```bash
sudo apt install terminator
terminator
```

| Keys | Action |
|------|--------|
| Ctrl+Shift+O | split horizontally |
| Ctrl+Shift+E | split vertically |
| Ctrl+Shift+W | close pane |
| Ctrl+Tab | next pane |
| Ctrl+Shift+X | zoom pane in/out |

## 1. Build and look at the robot

```bash
git clone <this repository> buzz_robot_arm
cd buzz_robot_arm
colcon build --symlink-install
source install/setup.bash
```

**Why `--symlink-install`.** `colcon build` copies every file from `src/` into
`install/`, and ROS runs from `install/`. So after a plain build, editing a
YAML or URDF in `src/` changes nothing until you build again. With
`--symlink-install` the files in `install/` are links to the ones in `src/`,
so an edit takes effect the next time you launch. You still have to build
once after adding a new file or package, and compiled code always needs a
rebuild. Everything in this lab is YAML, XML and Python, so after the first
build you mostly just relaunch.

Sanity check that the URDF parses and print its kinematic tree:

```bash
check_urdf src/buzz_arm_description/urdf/buzz_arm.urdf
```

Visualise it with sliders for every joint:

```bash
ros2 launch buzz_arm_description display.launch.py
```

That starts three nodes: `robot_state_publisher` (turns the URDF plus
`/joint_states` into `/tf`), `joint_state_publisher_gui` (the slider window),
and RViz with a RobotModel and a TF display already added. Move the sliders
and watch the frames. Open
[src/buzz_arm_description/launch/display.launch.py](src/buzz_arm_description/launch/display.launch.py)
to see how little it takes.

While that is running, in another pane:

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_tools view_frames        # writes frames_<date>.pdf in the current directory
ros2 run tf2_ros tf2_echo base_link Link_5
```

`tf2_echo` prints the transform from the first frame to the second as a
translation, a quaternion, and the same rotation as roll/pitch/yaw in radians
and degrees. Leave it running and move one slider at a time. Which joint
changes which numbers?

## 2. Read the URDF before you generate anything

Open [src/buzz_arm_description/urdf/buzz_arm.urdf](src/buzz_arm_description/urdf/buzz_arm.urdf).
It was exported from SolidWorks and then edited by hand. Find and understand:

- The link tree. Which link is the root? How many children does it have?
- The three parts of every link: `<visual>`, `<collision>`, `<inertial>`.
  Which of these does a motion planner use? Which would a physics simulator use?
- The `<limit>` element on each revolute joint. Where do you think the numbers
  came from? Why is `lower` always zero? Keep this one in mind: it comes back
  to bite in section 5.
- The joint named `Mimic_LFinger` and its `<mimic>` tag.
- The link named `floor`. It has a `<collision>` body but is a plain box, not
  a mesh. Why is it there?
- The `package://` prefix on every mesh path, and what in
  `CMakeLists.txt` makes it resolve.

## 3. Add a tool frame

**Do this before you open the Setup Assistant.** The assistant builds the
planning group from the frames that exist when you run it. If you add the
tool frame afterwards you will redo the assistant.

Look at the TF tree again. The last frames on the arm are `Link_5` and the
finger links. There is no frame at the point where the fingertips meet, which
is the point you actually care about when you grasp something. Industrial
robots call this the tool centre point, and ROS convention names the frame
`tool0`.

Add it to the URDF yourself: one new `<link name="tool0"/>` with no geometry,
and one new `<joint>` of type `fixed` whose parent is `Link_5` and whose child
is `tool0`. The distance from the `Link_5` origin to the closed fingertips is
**82 mm**. What you have to work out is *along which axis of `Link_5`*.

How to find it:

1. Launch `display.launch.py`, slide the gripper to fully closed, and look at
   the `Link_5` axes in the TF display next to the fingertips. One of them
   points out through the fingers.
2. Put `0.082` on that axis in the `<origin xyz>`, rebuild, relaunch, and
   check that the `tool0` frame sits between the closed fingertips.
3. Run `ros2 run tf2_ros tf2_echo base_link tool0` and move each joint in
   turn.

Keep the orientation at `rpy="0 0 0"` and note which way `tool0`'s Z axis
points. You will want to know when you plan to a pose later.

## 4. Run the MoveIt Setup Assistant

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Choose **Create New MoveIt Configuration Package** and load
`src/buzz_arm_description/urdf/buzz_arm.urdf`. Work through the tabs in order.
The assistant's own documentation is at
https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html

Guidance for this robot, tab by tab:

| Tab | What to do | Think about |
|-----|------------|-------------|
| Self-Collisions | Press Generate at the default density. Then switch to the **list view** (the matrix view only shows colours; the list prints the reason). Find every row whose reason is `Default` and involves `floor`, and **uncheck them**. Leave `Default` rows against `base_link` checked. | The four reasons: *Adjacent* = share a joint. *Default* = collide in the URDF's all-zeros pose. *Always* = collided in every random sample. *Never* = never collided in any sample. The generator always uses the all-zeros pose and nothing you set changes that. What does this arm look like at all zeros, and why does that make the `floor` rows wrong? |
| Virtual Joints | Add one fixed joint from `world` to `base_link`. | What happens to planning if you skip this? |
| Planning Groups | Arm group: use **Add Kin. Chain**, base `base_link`, tip `tool0`, solver `kdl_kinematics_plugin/KDLKinematicsPlugin`. Gripper group: use **Add Links** with the four finger links only, **no `tool0`**, kinematics solver **None**. | Should `Mimic_LFinger` be in a group? Why or why not? What changes if you build the arm group with Add Links instead of a chain? |
| Robot Poses | Add `home` for the arm (values below) and `open` / `closed` for the gripper. | Keep every value strictly inside the URDF limits. |
| End Effectors | Name the gripper group as the end effector, parent link `tool0`. | |
| Passive Joints | Leave empty. | This tab is for joints that exist in the URDF but no controller drives and no mimic rule computes, like a free caster wheel. Which joint on this arm is not driven, and why does it still not belong here? |
| ros2_control URDF Modifications | Add `position` command, `position` and `velocity` state. | Open the generated xacro afterwards. What hardware plugin did it pick? |
| ROS 2 Controllers | Press auto-add. Accept a `JointTrajectoryController` for both the arm and the gripper. | |
| MoveIt Controllers | Press auto-add. | Compare the two controller YAML files it writes. What must match between them? |
| Launch Files | Leave all of them checked. | `demo.launch.py` includes five of the others, so unchecking one of those breaks the demo. |
| Configuration Files | Fill in **your name and email**, otherwise the generated `package.xml` fails to build. Name the package `buzz_arm_moveit_config`, save it inside `src/`. | |

The `home` pose, in radians:

| Revolute_1 | Revolute_2 | Revolute_3 | Revolute_4 | Revolute_5 |
|-----------:|-----------:|-----------:|-----------:|-----------:|
| 0.0 | 1.5708 | 1.55334 | 1.58825 | 1.50098 |

Then build and launch:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch buzz_arm_moveit_config demo.launch.py
```

Do not expect it to plan yet. That is section 5.

## 5. Three things the assistant gets wrong

The generated package launches, but in Jazzy it does not plan and execute out
of the box. Three small edits are needed. Do not just apply them: for each
one, **first reproduce the failure, read what move_group prints, then fix it
and confirm.** Keep the terminal running `demo.launch.py` visible the whole
time. Each experiment takes a couple of minutes.

**Experiment A: the start pose.** Launch the demo and look at the arm in RViz
before touching anything. In the MotionPlanning panel pick `home` as the goal
and press Plan. Wait for it. Then look at
`config/initial_positions.yaml`: the assistant wrote `0` for every joint.
Remember what `lower="0"` means on this robot. Replace the zeros with the
`home` values, relaunch, plan again.

**Experiment B: acceleration limits.** With A fixed, plan to `home` and then
drag the arm somewhere else and plan again. Read the move_group terminal
carefully: the planner itself may report success a few milliseconds before
something else rejects the result. Then look at `config/joint_limits.yaml`.
For every joint set `has_acceleration_limits: true` and
`max_acceleration: 10.0`. Relaunch, plan again.

**Experiment C: execution.** With A and B fixed, press Plan & Execute. Note
what move_group printed at startup about the controllers, and what it prints
now. Then open `config/moveit_controllers.yaml`: under each controller add

```yaml
    action_ns: follow_joint_trajectory
    default: true
```

Relaunch, Plan & Execute. The simulated arm should now move.

Once all three work: drag the interactive marker (it sits at `tool0`, which
is why you added it), plan, execute. Open the **Scene Objects** tab, add a box
in the arm's path, and watch the planner route around it.

Optional: re-open your package in the Setup Assistant with
`ros2 launch buzz_arm_moveit_config setup_assistant.launch.py` to see where
each file came from.

## 6. Troubleshooting

- **Everything aborts after exactly 5 seconds with no reason in RViz.** That
  is RViz's default planning time running out. The reason is in the
  move_group terminal. Always.
- **Plans behave as if your edits were ignored.** Did you build with
  `--symlink-install`? Did you relaunch? move_group reads YAML once at
  startup, and `initial_positions.yaml` is baked into the robot description
  at launch.
- **Run one demo at a time.** Two `demo.launch.py` on one machine means two
  controller managers fighting; the second one's joint state broadcaster
  publishes zeros, and MoveIt plans from a start pose that is not what you
  see. Looks exactly like "my initial positions are ignored". Check with:

  ```bash
  ros2 node list                        # ghosts from a dead launch? -> ros2 daemon stop
  ros2 topic echo /joint_states --once  # should show the home values right after launch
  ros2 control list_controllers         # should show 3 controllers, all active
  ```

- **`Could not identify parent group for end-effector`.** `tool0` ended up in
  the gripper group. Remove it.
- **`Group 'gripper' is not a chain`** at every startup. You gave the gripper
  a KDL solver. Set it to None. Harmless, but noisy.

## 7. Deliverables

1. Your edited URDF with the `tool0` frame, and your generated and fixed
   `src/buzz_arm_moveit_config` package, committed to a branch named with
   your username.
2. A short write-up (one to two pages) answering, in your own words:
   - Which link is the root of the tree, and how is `floor` attached to it?
   - Which `Link_5` axis points out through the fingers, and how did you
     confirm `tool0` landed in the right place?
   - With `tf2_echo base_link tool0` running, first tilt the arm forward with
     `Revolute_2`, then move only `Revolute_1`. Which of the translation and
     roll/pitch/yaw numbers change, and why those? Then move only
     `Revolute_5`. What does that tell you about where `tool0` sits relative
     to the `Revolute_5` axis?
   - Which part of a `<link>` does the planner check for collisions, and what
     would change if the collision mesh were replaced by a simple cylinder?
   - Why is `Mimic_LFinger` absent from your planning groups and from the
     ros2_control block, and how does it still move in RViz?
   - Pick three rows of the self-collision list, one `Adjacent`, one `Never`,
     one `Default`, and explain why the assistant disabled each pair. Then
     explain why you re-enabled the `Default` rows against `floor`.
   - What does the virtual joint do, and what did you observe when you tried
     planning without it?
   - For each of experiments A, B and C: what did move_group print before the
     fix, and in one sentence, why does the fix work?
3. A screenshot of a successful plan around an obstacle you added.

## 8. On using AI tools

You may use any AI assistant you like, and you will almost certainly want to.
Two rules:

- Say so. List which tools you used and for what in your write-up.
- Include one concrete example of something an assistant told you that was
  wrong, incomplete, or did not work on your machine, and what you did about
  it. If you did not hit one, say what you did to check its answers.

The write-up questions and the obstacle screenshot are about what you observed
on your own screen. That is the part no assistant can do for you.

## What comes next

Part 2 uses your configuration to compare OMPL planners and reason about
collisions and path quality. Part 3 gives you a pick-and-place sequencer with
real bugs in it and asks you to capture your own waypoints and make it work.
