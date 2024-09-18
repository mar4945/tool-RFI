def get_config_JSON(config):
    
    return {
        "ATO_parameters": {
            "nu": config["nu"],
            "nx": config["nx"],
            "N_f": 17,
            "N_l": 65,
            "ts": 0.12,
            "ato_leader_ts": 0.24
        },
        "Communication_channel_parameters": {
            "lambda_exp": 0.1,
            "min_delay_time": 0.8,
            "p_channel": 0.8,
            "time_loss": 1700,
            "duration_loss": 7,
            "flag_loss": True
        },
        "train_parameters": {
            "M": 490000,
            "A": 26.152,
            "B": 8.365,
            "C": 1.914,
            "Tf": 1,
            "delta_param": 0.1,
            "pos_leader": 1400,
            "vel_leader": 50,
            "acc_leader": 0,
            "pos_follower": 0,
            "vel_follower": 50,
            "acc_follower": 0
        },
        "velocity_target": {
            "v_l_target": 50,
            "packet48": 1.02
        },
        "simulation_parameters": {
            "save_txt": True,
            "plot": True
        },
        "emergency_braking": -370000,
        "d_vc": 1000,
        "time_simulation": 2000,
        "ref_tau_1": 4800,
        "ref_tau_2": 4750,
        "ref_tau_3": 4700,
        "os1": False,
        "os2": True
    }