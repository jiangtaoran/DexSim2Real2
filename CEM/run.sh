#!/bin/bash

cd ManiSkill2-Learn/
python maniskill2_learn/apis/run_rl.py configs/mpc/cem/maniskill2_DigitalTwin_Allegro.py --gpu-ids 0 --dev
python maniskill2_learn/apis/run_rl.py configs/mpc/cem/maniskill2_DigitalTwin_Allegro.py --gpu-ids 0 --dev
