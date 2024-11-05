# Eigengrasp 

## Introduction 
We utilize EigenGrasp to reduce dexterous hand's DOF for more efficient trajectory searching in Sampling Based Predictive Control Module.

## File Structure

```
    main.py                     # train eigengrasp model
    eigengrasp.py               # provide utils for CEM module
    grasp_mat.npy               # hand pos generated with DexGraspNet
```

## Running EigenGrasp

### Data Generation
    
Generate enough grasp data in `DexGraspNet` and pack them in a single `*.npy` file, such as the `grasp_mat.npy` in this folder.
`grasp_mat.npy` contains 60800 grasp qpos generated on 474 objects, which is large enough in most cases.

### Train Eigengrasp Model

To train EigenGrasp:
```
python main.py --mode train --data_num -1 --dim 16 --output grasp_model_16.pkl --loss_result loss_result_16.csv
```
where ` dim ` represents remaining DOF of dexterous hand.




