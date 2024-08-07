from agent.Base_Agent import Base_Agent

# Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name
# host, agent_port , monitor_port , unum:int, team_name:str, enable_log, enable_draw
player = Base_Agent("localhost", 3100, 3200, 1, 1, "DudsForce")
# player2 = Base_Agent("localhost", 3100, 3200, 2, 1, "DudsForce")

w = player.world
# w2 = player2.world
while True:
    if player.behavior.is_ready("Get_Up"):
        player.behavior.execute_to_completion("Get_Up")
    player.behavior.execute("Run", w.ball_abs_pos[:2], True, None, True, 0.5)
        
    player.scom.commit_and_send( w.robot.get_command() )
    player.scom.receive()


    # if player2.behavior.is_ready("Get_Up"):
    #     player2.behavior.execute_to_completion("Get_Up")
    # player2.behavior.execute("Run", w2.ball_abs_pos[:2], True, None, True, 0.5)

        
    # player2.scom.commit_and_send( w2.robot.get_command() )
    # player2.scom.receive()
