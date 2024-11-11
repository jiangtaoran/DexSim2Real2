<div align='center'>

<h2>DexSim2Real<sup>2</sup> : Building Explicit World Model for Precise Articulated Object Dexterous Manipulation</h2>


[Taoran Jiang](https://github.com/jiangtaoran) <sup>\* </sup>, [Liqian Ma](https://github.com/TTimelord) <sup>\* </sup>, Yixuan Guan, Jiaojiao Meng, Weihang Chen, Zecui Zeng, Lusong Li, Dan Wu, Jing Xu, [Rui Chen](https://callmeray.github.io/homepage/)

Department of Mechanical Engineering, Tsinghua University

<sup>* </sup>Equal Contribution

</div>

<div align="center">

[![arXiv](https://img.shields.io/badge/arXiv-2409.08750-red)](https://arxiv.org/abs/2409.08750)
[![Project Page](https://img.shields.io/badge/Project-website-yellow
)](https://jiangtaoran.github.io/dexsim2real2_website)
[![Video](https://img.shields.io/badge/Video-youtube-orange
)](https://www.youtube.com/watch?v=gW9AHF2zDFY)


![teaser_video](./teaser.gif)

</div>

## Introduction

![frame](https://jiangtaoran.github.io/dexsim2real2_website/static/images/method.jpg)
Our DexSim2Real2 algorithm has 3 parts, **Interactive Perception**, **Explicit Physics Model Construction**, **Sampling Based Model Predictive Control**. 

For **Interactive Perception** module, We utilize [VRB](https://github.com/shikharbahl/vrb) and pixel warping method to acquire affordance. You can also try [where2act](https://github.com/daerduoCarey/where2act) method for interactive perception, which is also mentioned in our work.

For **Explicit Physics Model Construction** module, we use [Ditto](https://github.com/UT-Austin-RPL/Ditto) as our construction module. 

For **Sampling-based Model Predictive Control** Module,we choose [ManiSkill2](https://github.com/haosulab/ManiSkill2) and [ManiSkill2-Learn](https://github.com/haosulab/ManiSkill2-Learn) as the basic framework. Besides, we utilize EigenGrasp method to reduce the action dimension of dexterous hand. Corresponding can be found in the repo.

To complete the entire process, follow these steps:
1. capture the RGB image and depth information of the object with RGBD camera(we use RealSense D435i for experiment). Then use **VRB** to generate 3D contact point and post-contact vector.
2. drive the end effector to conduct the one-step interaction with the object within [Moveit](https://github.com/moveit/moveit).
3. Utilize **Ditto** to create world model of the object, saved as URDF file.
4. Utilize **CEM** to accurately manipulate the object in simulation.
5. another module is required to replay the trajectory on your real robot.

A more detailed `README.md` is provided for each module, outlining specific usage methods. Please refer to them for further information.
## Installation
This code has been tested on Ubuntu 20.04 with Cuda 11.6, Python3.8, and PyTorch 1.11.0.

To avoid potential package conflicts, it is recommended to create separate Python environments for each module. For each module, we have integrated a `requirements.txt` file to simplify the installation of required packages.

For example, in the case of CEM:
```
git clone https://github.com/jiangtaoran/DexSim2Real2.git
cd {parent_diectory_of_DexSim2Real2}
cd DexSim2Real2/CEM
pip install -e .
cd ManiSkill2-Learn/
pip install -e .
```
## Training and Planning in Simulation

### Ditto
Scripts for data collection are stored in `Ditto/scripts`.

Training: `python run.py experiment=all_stereo.yaml`

Generate digital twin: `python generate_digital_twin.py`
### CEM
Before starting planning, modify the paths of the config and the urdf file. You can modify parameters of CEM in `CEM/ManiSkill2-Learn/configs/mpc/cem/maniskill2_DigitalTwin_allegro.py`

You can modify the controller's config in `CEM/ManiSkill2/mani_skill2/assets/config_files/agents/fixed_xmate3_allegro.yml`.

You can modify the reward function in `CEM/ManiSkill2/mani_skill2/envs/CEM_allegro_env.py`.

Start planning by:

```
cd {parent_directory_of_DexSim2Real2}
cd DexSim2Real2/CEM/ManiSkill2-Learn
python maniskill2_learn/apis/run_rl.py configs/mpc/cem/maniskill2_DigitalTwin_Allegro.py --gpu-ids 0 --dev
```

Videos and trajectories are stored in `CEM/ManiSkill2-Learn/work_dirs`
## Citation
```
@misc{jiang2024dexsim2real2buildingexplicitworld,
      title={DexSim2Real$^{2}$: Building Explicit World Model for Precise Articulated Object Dexterous Manipulation}, 
      author={Taoran Jiang and Liqian Ma and Yixuan Guan and Jiaojiao Meng and Weihang Chen and Zecui Zeng and Lusong Li and Dan Wu and Jing Xu and Rui Chen},
      year={2024},
      eprint={2409.08750},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2409.08750}, 
}
```
Previous conference versio Sim2Real<sup>2 </sup>:
```
@INPROCEEDINGS{10160370,
  author={Ma, Liqian and Meng, Jiaojiao and Liu, Shuntao and Chen, Weihang and Xu, Jing and Chen, Rui},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Sim2Real2: Actively Building Explicit Physics Model for Precise Articulated Object Manipulation}, 
  year={2023},
  volume={},
  number={},
  pages={11698-11704},
  doi={10.1109/ICRA48891.2023.10160370}}
```
