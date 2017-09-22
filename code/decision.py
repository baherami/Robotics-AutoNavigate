import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
#def getSteer(angle,angles):
# A function to calculate the angle in other ways.
# angle is always Rover.steer and angles come from Rover.nav_angles	
# Solution one
#    idx = (np.abs(angles * 180/np.pi-angle)).argmin()
#    return np.clip(angles[idx],-15,15) 
def getSteer(angles,dist,rangles,rdist):

    if rangles is not None:
        mean_rock_angle=np.mean(rangles)
        idx=np.abs(angles- mean_rock_angle).argmin()
        possible_angles=angles[idx]*180/np.pi
        possible_distances=dist[idx]
            		
    elif angles is not None:
        cdist=np.where(dist<20)
        maAngles=np.ma.masked_outside(angles, -1, 45)
        maAngles.filled(0)
        possible_angles=(maAngles[cdist]* 180/np.pi)
        possible_distances=dist[cdist]
		#res= np.clip(np.mean(res),-15,15)
		#if(-16<res<16):
		#    res = -16
    
        if type(possible_angles) is not np.ma.core.MaskedArray:	
            possible_angles=None
            possible_distances=None
    #print(type(possible_angles))          		
    return possible_angles,possible_distances
    #elif(len(angles)==len(dist)) and (dist is not None and angles is not None):
	#	cdist=np.where(dist<20)
	#else:
	#	return -16
    
	#maAngles=np.ma.masked_outside(angles, -1, 45)
    #maAngles.filled(0)
    #if(cdist is not None):
    #    res=(maAngles[cdist]* 180/np.pi)
    #else :
    #        print("no value")
    #        return -16
		
    #    res= np.clip(np.mean(res),-15,15)
	#distmin=np.amin(angles-
        #print("everthing fine",res)
    #    if(-16<res<16): 
    #        return res#,np.mean(cdist)
    #    else:
    #        return -16
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
    if Rover.mode != 'backward':
        Rover.moveCount+=1	
    else:
        Rover.moveCount=-200
        Rover.stopCount=-100		
    #if navigator is stuck then do a reverse move		
    if (-0.2<Rover.vel<0.2):
        Rover.stopCount+=1
        if(Rover.stopCount>400):
            Rover.mode="Revive"
            print("Revive from stuck")
            Rover.stop_forward = 200 # Threshold to initiate stopping
            Rover.go_forward = 300 # Threshold to go forward again
        	 
            Rover.stopCount=0		
    if(Rover.moveCount>2000):
	    ##Finding out if the navigator starts to turn around itself or got stock in a point.
        if(Rover.prevPos!=None):
            ditanceFromPreviousPos=np.sqrt((Rover.pos[0]-Rover.prevPos[0])**2+(Rover.pos[1]-Rover.prevPos[1])**2)
            
            if ditanceFromPreviousPos<Rover.radious:			    
                Rover.mode="Revive"
                #Rover.steer = - Rover.steer
                Rover.samePositionCount=0
                print("Revive from circling")
                Rover.stopCount=0				
        Rover.prevPos=Rover.pos
        Rover.moveCount=0
        		
	
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode='stop'            
    
	
           
    # Check if we have vision data to make decisions with
    
    elif Rover.nav_angles is not None and Rover.nav_dists is not None:
        # Check for Rover.mode status
        possible_angles,possible_distances =getSteer(Rover.nav_angles,Rover.nav_dists,Rover.rock_angles,Rover.rock_dists)
        if( possible_angles is  None or possible_distances is None):
            print("no angle")
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            #print(distan)
			#if len(Rover.nav_angles) >= Rover.stop_forward:
            #print(len(possible_angles),len(possible_distances))			
            if( possible_angles is  None or possible_distances is None):
                print("no angle in forward")

            if len(possible_angles) >= Rover.stop_forward:
                #print("navigable",Rover.nav_angles)  			
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -5, 15)
                #print("before steer")
                Rover.steer =np.clip(np.mean(possible_angles),-15,15)
                #print("steer after call",Rover.steer)
				#print(Rover.steer," ",getSteer(Rover.nav_angles,Rover.nav_dists))
			# If there's a lack of navigable terrain pixels then go to 'stop' mode
            #elif len(Rover.nav_angles) < Rover.stop_forward:
            elif len(possible_angles) < Rover.stop_forward:   
				# Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = 2#Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
        elif Rover.mode=='Revive':
			# If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0.0
                Rover.brake = 2.0
                Rover.steer = 0
                Rover.mode = 'backward'
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                    Rover.throttle = -1.0
                    # Release the brake to allow turning
                    Rover.brake = 0.0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer =0 #np.amax(Rover.nav_angles * 180/np.pi)
                    Rover.mode = 'backward'
        if Rover.mode=='backward':
                    # Release the brake to allow turning
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    #Rover.steer =0 #np.amax(Rover.nav_angles * 180/np.pi)
                    Rover.throttle = -1.0
                    Rover.brake=0
                    Rover.backwardCount+=1
                    if(Rover.backwardCount>30):
                        Rover.backwardCount=0
                        Rover.mode = 'stop'
 		
        elif Rover.mode == 'to the rock': 
            # Check the extent of navigable terrain
            if(Rover.rock_dists is not None):
                meanRdist=np.mean(Rover.rock_dists)
            else:
                meanRdist=0
            #print("go rock",possible_distances,possible_angles,Rover.go_for_rock,possible_distances.any() >= Rover.go_for_rock)		

            if meanRdist>= Rover.go_for_rock:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < 1:
                    # Set throttle value to throttle setting
                    Rover.throttle = 0.1
					
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0         
                if(possible_angles is not None):
                    Rover.steer =np.clip(np.mean(possible_angles),-15,15)
                        #np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                else:
                    Rover.mode='stop'    						
				
            #np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
				#Rover.nav_dists.any() < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes! 
                print("no way to go")     					 
                Rover.throttle = 0
                    # Set brake to stored brake value
                Rover.brake = 1
                Rover.steer = 0
                Rover.mode = 'stop at rock'
		# If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop at rock':
            #print("rock stop",possible_distances,possible_angles)		
            # If we're in stop mode but still moving keep braking
            if(Rover.rock_dists is not None):
                meanRdist=np.mean(Rover.rock_dists)
            else:
                meanRdist=0
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = 1
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if meanRdist < Rover.stop_for_rock:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if meanRdist>= Rover.stop_for_rock:
                    # Set throttle back to stored value
                    Rover.throttle = 0.1#Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    if(possible_angles is not None):
                        Rover.steer =np.clip(np.mean(possible_angles),-15,15)# np.clip(np.mean(Rover.nav_angles * 180/np.pi), -5, 15)
                        Rover.mode = 'go for rock'
                    else:
                        Rover.mode='forward'
        
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            #distan=np.mean(Rover.nav_angles)
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(possible_angles)<Rover.go_forward:
                    Rover.throttle = 0
                    # Release the br	ake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15
					#Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(possible_angles)>=Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(possible_angles),-15,15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        #print("else")
        Rover.elseCounter+=1
        if(Rover.elseCounter>100):
            Rover.throttle = 0
            Rover.steer = 0
            Rover.brake = 0
            Rover.go_forward=20
            Rover.stop_forward=5   			
            Rover.stopCount=0
            Rover.elseCounter=0			
        else:
            Rover.throttle = 0
            Rover.steer = -15
            Rover.brake = 0
    if(Rover.mode!=Rover.prevMode):    
        print(">>>>>>>>>>>>>>>>>",Rover.mode)
        Rover.prevMode=Rover.mode
    if (Rover.fps!=Rover.prevFps):
        print("Current fps:",Rover.fps)
        Rover.prevFps=Rover.fps		
    return Rover

