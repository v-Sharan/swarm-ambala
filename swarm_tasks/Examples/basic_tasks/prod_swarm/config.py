import os
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class SwarmConfig:
    # Network Settings
    server_address: str = "127.0.0.1"
    heartbeat_timeout_sec: int = 28
    
    # Path/System Settings
    _cur_dir = os.path.dirname(os.path.abspath(__file__))
    _base_dir = os.path.dirname(_cur_dir)

    log_dir: str = os.path.join(_base_dir, "logs")
    search_grid_dir: str = os.path.join(_base_dir, "searchgrid")
    group_split_dir: str = os.path.join(_base_dir, "group_split")
    
    # Flight Parameters
    arrival_distance_threshold: float = 5.0
    bot_speed: float = 1.0  # Adjust according to physical test needs
    sleep_times: Dict[int, float] = None
    
    def __post_init__(self):
        if self.sleep_times is None:
            self.sleep_times = {
                1: 0.1, 2: 0.1, 3: 0.1, 4: 0.1, 
                5: 0.1, 6: 0.1, 7: 0.1, 8: 0.1
            }

# Global singleton configuration loader
CONFIG = SwarmConfig()
