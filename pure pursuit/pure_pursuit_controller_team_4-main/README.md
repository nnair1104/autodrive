# Pure Pursuit Controller - Freicar (Team 4)

1. Start the controller using 
   
   roslaunch freicar_control start_controller.launch
   
2. Publish the path using
 
   rosrun freicar_control pub_path.py
   
3. Spawn the car using 

   roslaunch freicar_agent sim_agent.launch name:=freicar_1 tf_name:=freicar_1 spawn/x:=0.43812255541599776 spawn/y:=0.8186631601507567 spawn/z:=0  sync_topic:=!
