from agent.Base_Agent import Base_Agent
import time
# Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name
# host, agent_port , monitor_port , unum:int, team_name:str, enable_log, enable_draw
player = Base_Agent("localhost", 3100, 3200, 1, 1, "DudsForce")


#Initialize standard behaviors
from behaviors.Poses import Poses

poses = Poses(player.world)

w = player.world
# w2 = player2.world
i = 0 
while True:



    # if player.behavior.is_ready("Get_Up"):
    #     player.behavior.execute_to_completion("Get_Up")
    # player.behavior.execute("Run", w.ball_abs_pos[:2], True, None, True, 0.5)
        
    # player.scom.commit_and_send( w.robot.get_command() )
    # player.scom.receive()

    if player.behavior.is_ready("Zero"): 
        player.behavior.execute_to_completion("Zero")
        print("Zero")
    time.sleep(2)
    if player.behavior.is_ready("Zero_Bent_Knees"):
        player.behavior.execute_to_completion("Zero_Bent_Knees")
        print("Zero_Bent_Knees")
    time.sleep(3)   
    time.sleep(5)
    

    print(i)
    i+= 1
    # if player2.behavior.is_ready("Get_Up"):
    #     player2.behavior.execute_to_completion("Get_Up")
    # player2.behavior.execute("Run", w2.ball_abs_pos[:2], True, None, True, 0.5)

        
    # player2.scom.commit_and_send( w2.robot.get_command() )
    # player2.scom.receive()
