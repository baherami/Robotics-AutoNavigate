import numpy as np
import math

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # If in a state where want to pickup a rock send pickup command
    
    #if Rover.prevYaw==None:
    #    Rover.prevYaw=Rover.yaw
    #    Rover.prevRoll=Rover.roll
    #else:
    #    f,wholePrevYaw = math.modf(Rover.prevYaw)
    #    f,wholePrevRoll=math.modf(Rover.prevRoll)
    #    f,wholeRoll=math.modf(Rover.roll)
    #    f,wholeYaw=math.modf(Rover.yaw)
    #    Rover.prevYaw=Rover.yaw
    #    Rover.prevRoll=Rover.roll
    #    if Rover.mode=="Revive":
    #        Rover.mode="stop"

    #    if wholePrevYaw==wholeYaw or wholePrevRoll==wholeRoll:
    #        Rover.sameNavigationCount+=1

    #if Rover.sameNavigationCount>100:
    #    Rover.throttle = Rover.throttle_set
    #    Rover.mode="Revive"
    #    Rover.sameNavigationCount=0
    #    print("Resteeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet")
		
		
	
	
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode='stop'
		
		
    # Check if we have vision data to make decisions with
    
    elif Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode=='Revive':
            #Rover.steer = np.random.randn()*30 - 15
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = -15 # Could be more clever here about which way to turn

        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = 20#Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        elif Rover.mode == 'to the rock': 
            # Check the extent of navigable terrain
			
            distan=np.mean(Rover.nav_dists)
            if distan >= Rover.go_for_rock:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < 1:#Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = 0.1
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15,15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif distan < Rover.stop_for_rock:
				#Rover.nav_dists.any() < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                    # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop at rock'
		# If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop at rock': 
            # If we're in stop mode but still moving keep braking
            distan=np.mean(Rover.nav_dists)
            if Rover.vel > 0.1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.1:
                # Now we're stopped and we have vision data to see if there's a path forward
                if (distan) < Rover.go_for_rock:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 0 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if distan >= Rover.go_for_rock:
                    # Set throttle back to stored value
                    Rover.throttle = 0.1#Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) ,-15,15)
                    Rover.mode = 'forward'
        
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    
    return Rover

