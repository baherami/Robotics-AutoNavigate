import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160),objectType="terrain"):
    #print("color_thresh") 
    # Create an array of zeros same xy size as img, but single channel
    if(img is None):
        print("invalid img") 	
        return None	
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if(objectType=='rocks'):
        imgBGR=cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        hsv = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([60,255,255])
        above_thresh = cv2.inRange(hsv, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(color_select,color_select, mask=above_thresh)
        color_select=np.copy(above_thresh)
 
    else: 
        above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                    & (img[:,:,1] > rgb_thresh[1]) \
                    & (img[:,:,2] > rgb_thresh[2])
        if(objectType=='obstacles'):
             above_thresh= np.logical_not(above_thresh)
        color_select[above_thresh]= 1
		
    # Index the array of zeros with the boolean array and set to 1
    
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    #print("rover_coords") 
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel,x_rock,y_rock,rocked):
    # Convert (x_pixel, y_pixel) to (dista[nce, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
	#print("to polar coords")
	if(len(x_pixel)<1 and len(y_pixel)<1):
		dist=None
		angles=None
	else:
		dist = np.sqrt(x_pixel**2 + y_pixel**2)
		# Calculate angle away from vertical for each pixel
		angles = np.arctan2(y_pixel, x_pixel)
	if rocked:
		if(len(x_rock)<1 and len(y_rock)<1):
			rdist=None
			rangles=None
		else:
			rock_angles = (np.arctan2(y_rock, x_rock))
		#np.resize(rock_angles,angles.shape)
			rangles=rock_angles
		#print(np.clip(np.mean(angles * 180/np.pi), -15, 15))
		#res=angles.copy()
		#leng=len(angles)
		#c=0
		#while(c<leng):
		#	if rock_angles.any()==angles[c]:
		#		res[c]=angles[c]
		#	else:
		#		res[c]=0
		#	c=c+1
		#if res.any()!=0:
		#	angles=res
		#else:
		#	angles=np.array([np.clip(np.mean(rock_angles* 180/np.pi), -15, 15),np.clip(np.mean(angles* 180/np.pi), -15, 15)])
			rdist = np.sqrt(x_rock**2 + y_rock**2)
	else :
		rdist=None
		rangles=None
	return dist, angles,rdist,rangles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    #print("rotatepix")
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    #print("translatepix")
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    #print("pixto world") 	
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    #print("persept transform")       
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
	#print("perception>step") 
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
	dst_size=5
	bottom_offset=0
	world_size=200
	scale=10
	source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
	destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])

    # 2) Apply perspective transform
	warped = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
	terrain = color_thresh(warped)
	obstacles = color_thresh(warped,objectType='obstacles')
	rocks = color_thresh(warped,objectType='rocks')
	# 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #           Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
	Rover.vision_image[:,:,0] =obstacles*250
	Rover.vision_image[:,:,2] =terrain*250
	Rover.vision_image[:,:,1] =rocks
	       
    
	# 5) Convert map image pixel values to rover-centric coords
	terrain_xpix, terrain_ypix = rover_coords(terrain)
	obstacles_xpix, obstacles_ypix = rover_coords(obstacles)
	rocks_xpix, rocks_ypix = rover_coords(rocks)
 
    # 6) Convert rover-centric pixel values to world coordinates
	terrain_world_x,terrain_world_y=pix_to_world(terrain_xpix, terrain_ypix , Rover.pos[0], Rover.pos[1],Rover.yaw, world_size, scale)
	obstacles_world_x,obstacles_world_y=pix_to_world(obstacles_xpix, obstacles_ypix , Rover.pos[0], Rover.pos[1],Rover.yaw, world_size, scale)
	rocks_world_x,rocks_world_y=pix_to_world(rocks_xpix, rocks_ypix , Rover.pos[0], Rover.pos[1],Rover.yaw, world_size, scale*100)

	# 7) Update Rover worldmap (to be displayed on right side of scree n)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
	if -0.3<Rover.pitch<0.3 and -0.3<Rover.roll<0.3:
		Rover.worldmap[terrain_world_y, terrain_world_x, 2] += 1
		Rover.worldmap[rocks_world_y, rocks_world_x, 1] += 1
		Rover.worldmap[obstacles_world_y, obstacles_world_x, 0] += 1
    
	# 8) Convert rover-centric pixel positions to polar coordinates
	rocked=False
	if (Rover.mode!='backward'):	
		if(len(rocks_world_x)>5):
			rocked=True
			Rover.mode='to the rock'
			#Rover.mode='forward' #temperory check
		elif Rover.mode=='to the rock':
			Rover.mode='forward'
		
	dist,ang,rdist,rang = to_polar_coords(terrain_xpix, terrain_ypix,rocks_xpix,rocks_ypix,rocked)
	
	# Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
	Rover.nav_dists = dist
	Rover.nav_angles = ang
	Rover.rock_dists = rdist
	Rover.rock_angles = rang
	
	#print(Rover.mode,np.mean(dist),np.clip(np.mean(ang * 180/np.pi),-100,100))
	return Rover