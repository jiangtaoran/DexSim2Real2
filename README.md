# DexSim2Real2
DexSim2Real 2 : Building Explicit World Model for Precise Articulated Object Dexterous Manipulation

![teaser_video](./teaser.gif)

[Website](https://jiangtaoran.github.io/dexsim2real2_website)|[arXiv](https://arxiv.org/abs/2409.08750)|[Video](https://www.youtube.com/watch?v=gW9AHF2zDFY)

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
