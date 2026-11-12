# Part 2: Planners and obstacles

In Part 1 you built a MoveIt configuration for the arm. In this lab you use it
to answer one question with data: **how do different motion planners behave
when something is in the way?**

Everything runs in simulation with the mock hardware from Part 1. Budget is
40 minutes. Steps 1 to 3 are the lab; step 4 is the write-up.

## What changed since Part 1

This lab runs on the package you built and fixed in Part 1. Four small
additions are needed, all listed in section 0:

- **A planner list.** The Setup Assistant in Jazzy does not write
  `config/ompl_planning.yaml`, so MoveIt has been running on a built-in
  default that exposes exactly one planner per group. Ask it for `RRT` by
  name and it silently plans with RRTConnect instead, with only a warning in
  the move_group terminal. This lab is about comparing planners, so you need
  the file.
- **A second named pose**, `reach_low`, so every plan in the lab goes
  between the same two configurations.
- **A new package**, `buzz_arm_planning_lab`, with two small scripts: one
  loads obstacles into the planning scene, one sends planning requests in a
  loop and records what comes back. One function in it is yours to write.
- **A reference `buzz_arm_moveit_config`** is included so the lab can be run
  without a Part 1 package. If you have your own, use yours and do not copy
  the reference over it. The reference is what Part 1 produces after the
  three fixes in its section 5, plus the two additions above.

## 0. Before the lab (10 minutes, at home)

1. Copy `src/buzz_arm_planning_lab/` and this file into the workspace from
   Part 1, next to your own `buzz_arm_moveit_config`.
2. Copy `src/buzz_arm_moveit_config/config/ompl_planning.yaml` from the
   reference package into your own package's `config/` folder, then read it.
   It is commented. It names the planning pipeline (request adapters, the
   OMPL plugin, response adapters), lists the OMPL planners you may ask for,
   and sets a default per group. If your arm group is not called `buzz_arm`,
   rename the `buzz_arm:` key near the bottom to match your SRDF.
3. Add one named pose to your SRDF, inside your arm group, next to `home`:

   ```xml
   <group_state name="reach_low" group="buzz_arm">
       <joint name="Revolute_1" value="0.0"/>
       <joint name="Revolute_2" value="2.6"/>
       <joint name="Revolute_3" value="1.55334"/>
       <joint name="Revolute_4" value="1.58825"/>
       <joint name="Revolute_5" value="1.50098"/>
   </group_state>
   ```

   If your arm group is not called `buzz_arm`, use your name there and pass
   `--group <name>` to the benchmark later.
4. Open `src/buzz_arm_planning_lab/buzz_arm_planning_lab/plan_benchmark.py`
   and read it top to bottom. It is short. One function, `path_length`, is
   left for you to write. It is marked `TODO` and the comment above it tells
   you what the trajectory message looks like. Three lines is enough. The
   script refuses to run until you do this, on purpose.
5. Build, then check that the planner list took:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch buzz_arm_moveit_config demo.launch.py
   ```

   In RViz's MotionPlanning panel, Context tab, there are two dropdowns:
   **Planning Pipeline** and, under it, **Planner**. Set the pipeline to
   `ompl`; the Planner dropdown should then list RRTConnect, RRT, RRTstar,
   PRM and a few more. If it shows only your group's name, move_group did
   not find your `ompl_planning.yaml`.
   The file has to be in `config/` and be named exactly that, and you have to
   relaunch after adding it.

## 1. See the obstacle (5 minutes)

Terminal 1:

```bash
ros2 launch buzz_arm_moveit_config demo.launch.py
```

Terminal 2:

```bash
ros2 run buzz_arm_planning_lab scene_objects list
ros2 run buzz_arm_planning_lab scene_objects load bar
```

A green bar appears in RViz across the front of the arm. It is now part of
move_group's *planning scene*: the planner treats it exactly like a link of
the robot it must not touch. Open
`src/buzz_arm_planning_lab/config/scenes.yaml` to see how it is described,
and skim `scene_objects.py` to see the one service call that put it there.
The Scene Objects tab in RViz does the same thing by hand.

## 2. Watch three planners (10 minutes)

In the RViz MotionPlanning panel:

- Planning tab, **Goal State** dropdown: pick `reach_low`.
- Context tab, **Planner** dropdown: pick `RRTConnect`. Back on the Planning
  tab press **Plan**. Watch the animated path. Press Plan three or four more
  times. Is it the same path each time?
- Change the planner to `RRT` and plan a few times. Then `RRTstar`. Notice
  the planning time shown in the panel for each.

Take a screenshot of one path from each planner for your write-up.

If one of these three is missing from the dropdown, your `ompl_planning.yaml`
does not list it under your group's `planner_configs`. Add it and relaunch.

## 3. Measure instead of watching (15 minutes)

The benchmark sends the same request RViz sends, many times, and records what
comes back. Free space first:

```bash
ros2 run buzz_arm_planning_lab scene_objects clear
ros2 run buzz_arm_planning_lab plan_benchmark --goal reach_low \
    --planners RRTConnect RRT RRTstar PRM --trials 5 --time 2 \
    --label free --out results.csv
```

Then the bar:

```bash
ros2 run buzz_arm_planning_lab scene_objects load bar
ros2 run buzz_arm_planning_lab plan_benchmark --goal reach_low \
    --planners RRTConnect RRT RRTstar PRM --trials 5 --time 2 \
    --label bar --out results.csv
```

Each run ends with a summary. Copy the numbers into this table. A success
count below 5 is a result, not a mistake: write it down and keep going.

| scene | planner | success | mean planning time (s) | mean path length (rad) |
|-------|---------|---------|------------------------|------------------------|
| free  | RRTConnect | /5 | | |
| free  | RRT        | /5 | | |
| free  | RRTstar    | /5 | | |
| free  | PRM        | /5 | | |
| bar   | RRTConnect | /5 | | |
| bar   | RRT        | /5 | | |
| bar   | RRTstar    | /5 | | |
| bar   | PRM        | /5 | | |

If you have time left, load `overhang` and add four more rows. Going over
the bar is no longer allowed there.

To see the same data as charts (success rate, every trial's planning time
on a log scale, every trial's path length, one column per scene):

```bash
sudo apt install python3-matplotlib      # once
ros2 run buzz_arm_planning_lab plot_results results.csv
```

It writes `results_charts.png` next to the CSV and prints the summary table
again. Include the PNG in your write-up.

Add `--execute` to any benchmark command to make the simulated arm actually
move through each plan in RViz instead of only planning.

## Troubleshooting

- **Every planner reports the same ~10 ms and the same path length, and
  RRTstar does not use its time budget.** Either the Planning Pipeline
  dropdown is not on `ompl`, or move_group is ignoring the planner name.
  Look in its terminal for
  `Cannot find planning configuration for group 'buzz_arm' using planner
  'RRT'. Will use defaults instead.` and go back to section 0, step 2.
- **A trial fails, but the move_group terminal shows no `Planning timed
  out`.** Look for `Computed path is not valid` followed by a `Found a
  contact between ...` line. The planner found a path; a later stage (the
  `Calling PlanningResponseAdapter` line just above names it) re-checked it
  after time parameterization and rejected it. It happens to roughly one
  plan in twenty and is not your fault.
- **`No group_state 'reach_low'`.** The benchmark reads the SRDF that
  move_group loaded. Did you relaunch after editing it?

## 4. Write-up

Hand in `results.csv`, the filled-in table, your three screenshots, and
short answers to these, in your own words:

1. What is `path_length` measuring, and why is it almost the same for every
   planner in the `free` rows (no obstacle loaded)?
2. Every successful RRTstar trial reports a planning time equal to the
   budget you gave it. The others finish in milliseconds. Why? What did
   RRTstar do with the rest of the time, and can you see it in the path
   length column of the `bar` rows?
   Now look at RRTstar's success count with the bar. RRT grows one tree
   toward random samples and solves this in well under a second; RRTstar
   grows the same kind of tree with the same budget and sometimes finds no
   path at all (`Planning timed out` in the move_group terminal, and the
   benchmark prints `plan=0.0s`). What extra work does RRTstar do on every
   extension that RRT does not, and why does that cost it the budget on a
   problem RRT finds easy?
3. RRT's planning time varies far more than RRTConnect's between trials of
   the same problem. Both are random. What is different about how they grow
   their trees?
4. Run the `bar` benchmark again with `--time 0.2`. Which planners lose
   successes, and what does that tell you about choosing a time budget on a
   real robot?
5. Put the obstacle somewhere the planner cannot avoid: edit `scenes.yaml`
   so the bar sits at z = 0.15 and reload it. What does move_group say, and
   at what stage of the pipeline is the request rejected? (The answer is in
   the terminal running the demo, not in the benchmark output.) Put the bar
   back to z = 0.33 when you are done.
6. One thing that went wrong today and how you found out what it was.

## On using AI tools

Same rule as Part 1: say which tools you used and for what, and include one
concrete thing an assistant told you that was wrong or did not match what
you saw on screen. Questions 2 to 5 are about numbers from your own runs.
