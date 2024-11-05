# Sampling Based Model Predicitive Control

## File Structure

The code is based on the ManiSkill2 framework. The following files need to be edited:

`CEM/ManiSkill2/mani_skill2/agents`: Agent classes (description for agent, such as links).

`CEM/ManiSkill2/mani_skill2/agents/controllers`: Controller classes.

`CEM/ManiSkill2/mani_skill2/assets/config_files/agents`: Agent configs (including joints, controllers and so on).

`CEM/ManiSkill2/mani_skill2/envs`: Environments classes 

(`CEM_env.py` for robot with a gripper, `CEM_env_allegro.py` for robot with an allegro hand, `fixed_xmate3_allegro_env.py` lies the base class for `CEM_env_allegro.py`).

`CEM/ManiSkill2-Learn/configs/mpc/cem`: Configs for environments (`maniskill2_DigitalTwin_Allegro.py` for CEM_env_allegro).

`CEM/ManiSkill2-Learn/work_dirs`: Results and output files for CEM.



## Running CEM
### Preparation
1. Run `Ditto` to generate digital twin.
   
2. Run `Eigengrasp` to generate eigengrasp model.
   
3. Edit eigengrasp configs in `CEM_env_allegro.py` to use eigengrasp, `eigen_dim` must as same as the model you load.

### Planning
Start planning by running: 
```
cd Sim2Real-work/CEM/Maniskill2-Learn
python maniskill2_learn/apis/run_rl.py config/mpc/cem/maniskill_DigitalTwin_Allegro.py --gpu-ids 0 --dev
```
### Result
In `CEM/ManiSkill2-Learn/work_dirs/CEM-allegro-v0/`, you can see videos in `videos/0.mp4` and reward values in `statistic.csv`.

