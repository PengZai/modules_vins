# Module Visual Inertial Navigation System(MVINS)

There are many advanced VIO or VINS system, and they are avaiable, high performance and robustness. However, they were designed for performance so that these systems are usually complicated and tight coupled, which make it difficult to grasp and compare what role and capbility are for each functional modules, such as feature detections, tracking/data association, state initialization, mapping, related backend optimization and so on. Here, we highly modulize a Visual Inertial Navigation System(module_vins) with decoupled modualization and high abstract interface design, then we could swtich functional module in VSLAM sytem easily. 

Not that only, we also are dedicated to provided adoptors for connecting well-designed abstract interface to modern SLAM project or other awesome open source tools, which could enrich the function of module VINS and support following research works.


# Setup
```bash
mkdir -p modules_vins/catkin_ws/src
cd modules_vins/catkin_ws/src
git clone this repository
run ./modules_vins/Docker/run_dev.sh


# setup your dataset path in variable DATA_PATH in modules_vins/Docker/run_dev.sh

```

# Datasets & Benchmark





# EVALUATION

we use [evo tools]{https://github.com/MichaelGrupp/evo} for evaluation. So we only provide the output trajetories file with TUM format for evaluation.

# License

This project is based on [open vins]{https://github.com/rpng/open_vins/} and ORB3-SLAM{https://github.com/UZ-SLAMLab/ORB_SLAM3}. Thanks for their excellent works for Visual SLAM commuities.