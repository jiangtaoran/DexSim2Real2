<<<<<<< HEAD
# CEM for trajectory generation in Sim2Real

## Files
Only introduce files that you may need to edit in most cases.  

- `CEM/ManiSkill2/mani_skill2/agents`: Agent classes (description for agent, such as links).
- `CEM/ManiSkill2/mani_skill2/agents/controllers`: Controller classes.
- `CEM/ManiSkill2/mani_skill2/assets/config_files/agents`: Agent configs (including joints, controllers and so on).
- `CEM/ManiSkill2/mani_skill2/envs`: Environments classes (`CEM_env.py` for robot with a gripper, `CEM_env_allegro.py` for robot with an allegro hand, `fixed_xmate3_allegro_env.py` lies the base class for `CEM_env_allegro.py`).
- `CEM/ManiSkill2-Learn/configs/mpc/cem`: Configs for environments (`maniskill2_DigitalTwin_Allegro.py` for CEM_env_allegro).
- `CEM/ManiSkill2-Learn/work_dirs`: Results and output files for CEM.

## Before Running
Run Ditto to generate digital twin.
Run Eigengrasp to generate eigengrasp model.
Edit `maniskill2_DigitalTwin_Allegro.py` following instructions.
Edit eigengrasp configs in `CEM_env_allegro.py` to use eigengrasp, `eigen_dim` must as same as the model you load.

## Run 

Start planning by running 
```bash
conda activate CEM
cd Sim2Real-work/CEM/Maniskill2-Learn
python maniskill2-learn/apis/run_rl.py config/mpc/cem/maniskill_DigitalTwin_Allegro.py --gpu-ids 0 --dev
```

## Result
In `CEM/ManiSkill2-Learn/work_dirs/CEM-allegro-v0/`, you can see videos in `videos/0.mp4` and reward values in `statistic.csv`.

