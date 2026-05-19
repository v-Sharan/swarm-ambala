goal_table = []
master_num = -1
master_flag = False

disperse_multiple_goals = []
start_multiple_goals = []
return_multiple_goals = []
goal_points = []
agg_goal_point = []
origin = []
removed_uav_homepos_array = []
bot_speed = 3.0

master_num = 0
master_flag = True

nextwaypoint = 0


port_dict = {
    1: 14551,
    2: 14552,
    3: 14553,
    4: 14554,
    5: 14555,
    6: 14556,
    7: 14557,
    8: 14558,
}

sleep_times = {
    8: 0.0000001,
    7: 0.0000001,
    6: 0.0000001,
    5: 0.0000001,
    4: 0.0000001,
    3: 0.0000001,
    2: 0.0000001,
    1: 0.0000001,
}


goal_path_csv_array = []
goal_path_csv_array_flag = False
skip_wp_flag = False
next_wp = 0

index = ""

def update_index(new_index):
    global index
    index = new_index

def get_index():
    global index
    return index