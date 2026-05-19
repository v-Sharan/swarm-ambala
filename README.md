Swarmbots - Tasks
🛩️ SwarmUAV: Decentralized Motion Planning for Fixed-Wing Drones
🧠 Overview

SwarmUAV implements a decentralized swarm coordination system for fixed-wing UAVs. It supports real-time decision-making based on local sensing and Artificial Potential Fields (APFs) to enable smooth navigation, obstacle avoidance, and group behaviors without central control.

Each UAV:

    Operates using only local sensor data
    Follows attractive and repulsive virtual forces
    Makes independent motion decisions
    Maintains separation and formation without direct communication

⚙️ Key Features

    ✅ Aggregation: Regroup UAVs in flight
    ✅ Dispersion: Spread swarm for wide-area coverage
    ✅ Goal Navigation: Move toward dynamically assigned goals
    ✅ Split Search: Distribute UAVs across multiple locations
    ✅ Scalable & Decentralized: Works with 3 or 300+ UAVs

🧮 Theory: Artificial Potential Fields

Swarm coordination is based on the Artificial Potential Field method:

    🟢 Attractive Forces: Pull UAVs toward waypoints or mission zones
    🔴 Repulsive Forces: Push UAVs away from nearby agents or obstacles
    🧭 Resultant Vector: Each UAV computes a net motion direction using local forces

This results in emergent group behavior without centralized control or global communication.
🚀 Installation

Clone the repo and install the package locally:

git clone https://github.com/NithyaSriVenkatesh/fixed_wing_swarm

Install the package from the root directory as follows:

cd swarm_tasks/Examples/basic_tasks python3 fixed_wing.py
