import os.path

log_level = "INFO"

agent_cfg = dict(
    type="CEM",
    cem_cfg=dict(
        n_iter=2,
        population=300, # laptop and faucet: 300, drawer: 300 or 600
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
    horizon=50,
)

DIGITAL_TWIN_CONFIG_DIR = f'/home/rvsa/ditto_2/digital_twin/'
env_cfg = dict(
    type="gym",
    env_name="CEM-v0",
    articulation_config_path=os.path.join(DIGITAL_TWIN_CONFIG_DIR, 'drawer_video_123.yaml'),
    unwrapped=False,
    obs_mode="state_dict",
    reward_mode="dense",
    reward_scale=0.3,
    control_mode="pd_joint_delta_pos",
    # control_mode='pd_ee_twist',
    # control_mode='pd_ee_delta_pos',
    use_cost=False,
    # vhacd_mode="new",
    horizon=40,
)

rollout_cfg = dict(
    type="Rollout",
    num_procs=10,
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