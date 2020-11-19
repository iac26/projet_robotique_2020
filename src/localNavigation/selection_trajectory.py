import math
global_trajectory = [[149, 286], [260.0, 294.0], [319.0, 272.0], [319.0, 242.0], [286.0, 203.0], [280.0, 179.0], [277.0, 151.0], [219.0, 165.0], [202.0, 77.0], [347.0, 52.0], [468.0, 50.0], [508.0, 69.0], [501.0, 140.0], [496.0, 171.0], [352.0, 
204.0], [319.0, 242.0], [260.0, 294.0], [149.0, 286.0]]
TOLERENCE_POSITION = 2
count_trajectory=0

def detect_trajectory(actual_position): 
    global goal_actual
    if (abs(goal_actual[0]-actual_position[0])<=TOLERENCE_POSITION) and (abs(goal_actual[1]-actual_position[1])<=TOLERENCE_POSITION):
        return True


def change_goal(global_trajectory,actual_position):
    if detect_trajectory(actual_position):
        global count_trajectory
        count_trajectory += 1
        global goal_actual
        goal_actual=global_trajectory[count_trajectory]


while True:
    s=input().split()
    print("x_a: ",float(s[0]))
    print("y_a: ",float(s[1]))

    actual_position=[int(s[0]),int(s[1])]
    goal_actual=global_trajectory[count_trajectory]
    change_goal(global_trajectory,actual_position)
    #print("actual position: ",actual_position)
   # print("actual goal: ",goal_actual)


