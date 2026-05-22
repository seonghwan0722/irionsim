import gymnasium as gym
from .complete_asm_env_cfg import CompleteAsmEnvCfg
from .complete_asm_agent_cfg import CompleteAsmPPORunnerCfg

# Register the task
gym.register(
    id="Isaac-CompleteAsm-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": CompleteAsmEnvCfg,
        "rsl_rl_cfg_entry_point": CompleteAsmPPORunnerCfg,
    },
)
