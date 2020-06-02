#!/usr/bin/env python3

from ppadb.client import Client
from PIL import Image
import numpy as np
import time
import math

	
def getPixelInformation():
    #This function takes a device screencap and converts it into a numpy array.  Returning a image that is in (y,x) coordinates.
    # Also known as (height, width)
    
    image = device.screencap()
    with open("game.png", 'wb') as file:
        file.write(image)
        
    image = Image.open('game.png')
    image = np.array(image, dtype=np.uint8)  #convert image into numpy array for easily accessing pixel data. 
                                             # In (y,x) coordinate system instead of (x,y)
    
    image = image[600:2200] #Take only the rows that matter on the screen
    
    return image
	
	
def checkForCorrectColor(currentPixel):
    #Checks if the pixel is rgb color for the rims on the screen
    #Red rim:  216 79 44
    #Gray rim: 170 170 170
    
    if(currentPixel[0] > 210 and currentPixel[0] < 222 and currentPixel[1] > 73 and currentPixel[1] < 85 and currentPixel[2] > 38 and currentPixel[2] < 50):
        #Red rim pixel
        return True
    if(currentPixel[0] == 170 and currentPixel[1] == 170 and currentPixel[2] == 170):
        #Gray rim pixel
        return True
    return False
    
    
def findFirstPixelOccurence(rgbImage):
    #This function finds the location of the first occurence of a pixel with a hoop color.

    firstPixelLocation = [0,0]  #This is the pixel location of the first occurence of a hoop color.  These are given in x,y coordinates
    foundPixel = False

    i = 0
    while i < len(rgbImage):  #Loop through length of screen
        if(foundPixel):
            break
    
        for j in range(len(rgbImage[0])):  #Loop through width of screen
            currentPixel = rgbImage[i][j]
            
            if(checkForCorrectColor(currentPixel)):
                #Current pixel is the correct rgb color for the rims on the screen
                firstPixelLocation = [j,i]
                foundPixel = True
                break
                
        i += 8 #This is the number of rows to skip when reading pixels
    
    return firstPixelLocation
    
    
def checkForMovingHoop():
    #Checks if the hoop to aim at is moving.  It does that by taking an image and getting the hoop location.  It then takes another image
    # and compares the new hoop location with the old.  True means the hoop is moving.
    
    #Do first check
    rgbImage = getPixelInformation()
    firstCheckLocation = findFirstPixelOccurence(rgbImage)  #This is the pixel location on the first check.  These are given in x,y coordinates
    
    time.sleep(.3)
    
    #Do second check
    rgbImage = getPixelInformation()
    secondCheckLocation = findFirstPixelOccurence(rgbImage)  #This is the pixel location on the second check.  These are given in x,y coordinates
    
    #Check if hoops are in the same location
    if(firstCheckLocation != secondCheckLocation):
        #Hoop has moved
        return True
    
    #Hoop hasn't moved
    return False
    
    
def findMiddleOfHoops(rgbImage):
    #This function works by finding pixels on the rims of the hoop and averaging the coordinates to 
    #find the middle of the hoop.
    
    aimHoopLocation = [0,0]  #Both of these are given in x,y coordinates
    ballHoopLocation = [0,0]
    numOfPixelsInFirstHoop = 0  #Number of red pixels found in first hoop
    numOfPixelsInSecondHoop = 0  #Number of red pixels found in second hoop
    numOfHoopsCompleted = 0  #Number of hoops with all pixels found
    foundHoop = False  #Tells the program a hoop has been found
    rowHasRim = False  #Tells the program the row has rim pixels
    
    i = 0
    while i < len(rgbImage):  #Loop through length of screen
        if((not rowHasRim) and foundHoop):
            #Move to next hoop if at end of current hoop
            numOfHoopsCompleted += 1
            foundHoop = False
        
        if(numOfHoopsCompleted == 2):
            #Exit loop after two hoops are found
            break
        
        rowHasRim = False
        for j in range(len(rgbImage[0])):  #Loop through width of screen
            currentPixel = rgbImage[i][j]
            
            if(checkForCorrectColor(currentPixel)):
                #Current pixel is the correct rgb color for the rims on the screen
                if(not foundHoop):
                    #Found a hoop
                    foundHoop = True
                
                if(not rowHasRim):
                    #Current row has part of the rim
                    rowHasRim = True
                
                if(numOfHoopsCompleted == 0):
                    #Adds pixel's coordinates to the hoop to aim at
                    aimHoopLocation[0] += j
                    aimHoopLocation[1] += i
                    numOfPixelsInFirstHoop += 1
                else:
                    #Adds pixel's coordinates of the ball hoop
                    ballHoopLocation[0] += j
                    ballHoopLocation[1] += i
                    numOfPixelsInSecondHoop += 1
        i += 3 #This is the number of rows to skip when reading pixels
    
    #This is where the coodinates are averaged with the number of pixels found
    aimHoopLocation[0] = int(aimHoopLocation[0] / numOfPixelsInFirstHoop)
    aimHoopLocation[1] = int(aimHoopLocation[1] / numOfPixelsInFirstHoop) + 82
    ballHoopLocation[0] = int(ballHoopLocation[0] / numOfPixelsInSecondHoop)
    ballHoopLocation[1] = int(ballHoopLocation[1] / numOfPixelsInSecondHoop) + 82
    
    return aimHoopLocation, ballHoopLocation
    
	
def calcPullback(aimHoopLocation, ballHoopLocation):
    #This function calculates how far to pull the ball and what direction.  It works by calculating what velocity the ball
    # must start at to reach the closest corner of the hoop to aim at.  These velocities are then used to find the angle to 
    # throw at.
    
    #Determine where to aim.  The aiming spot is at the corner of the hoop.
    if(aimHoopLocation[0] - ballHoopLocation[0] >= 0):
        peakThrowLocation = [aimHoopLocation[0] - 140, aimHoopLocation[1] - 222]
    else:
        peakThrowLocation = [aimHoopLocation[0] + 140, aimHoopLocation[1] - 222]
    
    accel = 2872  #This is the acceleration due to gravity in pixels/sec^2
    delX = peakThrowLocation[0] - ballHoopLocation[0]  #change in x position
    delY = peakThrowLocation[1] - ballHoopLocation[1]  #change in y position
    
    #Calculat initial velocity needed to have peak of throw at the corner of the hoop
    yVel = -math.sqrt(-2 * accel * delY)
    
    #Time in air to said point
    time = -yVel/accel
    
    #Velocity needed to reach point in given time
    xVel = delX / time
    
    #Calculate throwing angle with respect to the vertical
    throwAngle = math.atan(xVel/yVel)
    print("Angle: " + str(math.degrees(throwAngle)))
    
    #Calculate how far to pull back on screen
    yPullBack = delY / 1.7
    #yPullBack = yVel * (.2474 - (.0740 / 800) * (2872 + yVel))
    #yPullBack = yPullBack
    xPullBack = math.tan(throwAngle) * yPullBack * .8
    
    return xPullBack, yPullBack
    
    
def calcPullbackBigThrow(aimHoopLocation, ballHoopLocation):
    #This function calculates how far to pull the ball and what direction.  It works by throwing the ball at a set y 
    # velocity and using that to calculate the required x velocity.  This is then used to find the angle to drag the ball.
    
    yVel = -2800 #How hard to throw the ball in the y direction in
    #yVel = -2733
    accel = 2872  #This is the acceleration due to gravity in pixels/sec^2
    delX = aimHoopLocation[0] - ballHoopLocation[0]  #change in x position
    delY = aimHoopLocation[1] - ballHoopLocation[1]  #change in y position
    
    #Calculate the final y velocity before hitting the hoop to be aimed at
    yVelFinal = math.sqrt(yVel ** 2 + 2 * accel * delY)
    
    #Calculate time in air using y final velocity
    time = (yVelFinal - yVel) / accel
    
    #Calculate initial x velocity with time in air
    xVel = delX / time
    
    #Calculate throwing angle with respect to the vertical
    throwAngle = math.atan(xVel/yVel)
    print("Angle: " + str(math.degrees(throwAngle)))
    
    #Calculate how far to pull back on screen
    yPullBack = -700
    #yPullBack = -676
    xPullBack = math.tan(throwAngle) * yPullBack
    
    return xPullBack, yPullBack
    
	
def makeShot():
    #This function puts all parts of the program together.  It figures out where the hoops are located and then
    # determines how and where to throw the ball.
    
    isMoving = checkForMovingHoop()
    print("Moving: " + str(isMoving))
    
    #Get pixel information on screen
    rgbImage = getPixelInformation() #This image is in y,x coordinates not x,y
	
    #Find Hoop Locations
    aimHoopLocation, ballHoopLocation = findMiddleOfHoops(rgbImage) #These locations are in x,y coordinates now
    print("Aim hoop location: x: " + str(aimHoopLocation[0]) + " y: " + str(aimHoopLocation[1]))
    print("Ball hoop location: x: " + str(ballHoopLocation[0]) + " y: " + str(ballHoopLocation[1]))
    
    #Calculate the distance in pixels to pull the ball to throw it into the hoop
    if(isMoving):
        #Throw ball really high if moving
        xPullBack, yPullBack = calcPullbackBigThrow(aimHoopLocation, ballHoopLocation)
    else:
        #Throw ball normally if not moving
        xPullBack, yPullBack = calcPullback(aimHoopLocation, ballHoopLocation)
    print("xPullBack: " + str(xPullBack))
    print("yPullBack: " + str(yPullBack))
    
    #Issue swipe command to the phone with given pull back distances
    #device.shell("input touchscreen swipe x1 y1 x2 y2 dur")
    command = "input touchscreen swipe 700 800 " + str(700 - xPullBack) + " " + str(800 - yPullBack) + " 500"
    device.shell(command)
	

abd = Client(host="127.0.0.1", port=5037)
devices = abd.devices()

if (len(devices) == 0):
    print("no devices")
else:
    device = devices[0] #Assume first device is the device needed
	
    #command = "input touchscreen swipe 700 800 700 " + str(800 + 331) + " 1500"
    #print(command)
    #device.shell(command)
    for i in range(20):
        print("")
        print("Shot " + str(i + 1))
        makeShot()
        time.sleep(4)