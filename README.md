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

For **Interactive Perception** module, We utilize [VRB](https://github.com/shikharbahl/vrb) and pixel warping method to acquire affordance.

For **Explicit Physics Model Construction** module, we use [Ditto](https://github.com/UT-Austin-RPL/Ditto) as our construction module. 

For **Sampling-based Model Predictive Control** Module,we choose [ManiSkill2](https://github.com/haosulab/ManiSkill2) and [ManiSkill2-Learn](https://github.com/haosulab/ManiSkill2-Learn) as the basic framework. Besides, we utilize EigenGrasp method to reduce the action dimension of dexterous hand. Corresponding can be found in the repo.

You can also try [where2act](https://github.com/daerduoCarey/where2act) method for interactive perception, which is also mentioned in our work.


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
Previous conference version of our work Sim2Real<sup>2 </sup>:
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
