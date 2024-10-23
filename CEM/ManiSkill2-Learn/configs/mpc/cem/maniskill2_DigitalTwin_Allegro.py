import os.path

log_level = "INFO"

agent_cfg = dict(
    type="CEM",
    cem_cfg=dict(
        n_iter=2,
        # Edit population to change the number of samples
        # Smaller population size will lead to faster training but worse performance
        population=600,     # laptop and faucet: 300, drawer: 300 or 600
        elite=20,
        lr=1.0,
        temperature=1.0,
        use_softmax=False,
        add_histroy_elites=True
    ),
    add_actions=True,
    action_horizon=1,
    scheduler_config=dict(
        type="KeyStepScheduler",
        keys=["population", "n_iter"],
        gammas=1,
        steps=15,
    ),
    horizon=10,
)
# edit the following 2 lines to change the object type and index
obj_type = 'drawer'
obj_index = 1
DIGITAL_TWIN_CONFIG_DIR = f'/data/guest2_documents/work/Sim2Real2-work/ditto/real_test/real_datasets_allegro/{obj_type}/video_{obj_index}/digital_twin/'
env_cfg = dict(
    type="gym",
    env_name="CEM-allegro-v0",
    articulation_config_path=os.path.join(DIGITAL_TWIN_CONFIG_DIR, f'{obj_type}_video_{obj_index}.yaml'),
    unwrapped=False,
    obs_mode="state_dict",
    reward_mode="dense",
    reward_scale=0.3,
    control_mode="pd_joint_delta_pos",
    # control_mode='pd_ee_twist',
    # control_mode='pd_ee_delta_pos',
    use_cost=False,
    # vhacd_mode="new",
    horizon=50,
)

rollout_cfg = dict(
    type="Rollout",
    num_procs=20,
    shared_memory=True,
)


eval_cfg = dict(
    type="Evaluation",
    num_procs=1,
    horizon=30,
    use_hidden_state=True,
    start_state=None,
    save_traj=True,
    save_video=True,
    use_log=True,
    # save_info=True,
    log_every_step=True,
)
