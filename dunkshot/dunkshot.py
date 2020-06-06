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
    
    
def findFirstPixelOccurence():
    #This function finds the location of the first occurence of a pixel with a hoop color.

    rgbImage = getPixelInformation()

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
                
        i += 80 #This is the number of rows to skip when reading pixels
    
    return firstPixelLocation
    
    
def checkForMovingHoop():
    #Checks if the hoop to aim at is moving.  It does that by taking an image and getting the hoop location.  It then takes another image
    # and compares the new hoop location with the old.  True means the hoop is moving.
    
    #Do first check
    firstCheckLocation = findFirstPixelOccurence()  #This is the pixel location on the first check.  These are given in x,y coordinates
    
    time.sleep(.4)
    
    #Do second check
    secondCheckLocation = findFirstPixelOccurence()  #This is the pixel location on the second check.  These are given in x,y coordinates
    
    #Check if hoops are moving and if so what direction
    if(firstCheckLocation[1] == secondCheckLocation[1] and firstCheckLocation[0] != secondCheckLocation[0]):
        #Moving horizontally
        return "horizontal"
    elif(firstCheckLocation != secondCheckLocation):
        #Moving vertically
        return "vertical"
    
    #Hoop hasn't moved
    return "no movement"
    
    
def findMiddleOfHoops():
    #This function works by finding pixels on the rims of the hoop and averaging the coordinates to 
    #find the middle of the hoop.

    #Get pixel information on screen
    rgbImage = getPixelInformation() #This image is in y,x coordinates not x,y
    
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
        #i += 18 #This is the number of rows to skip when reading pixels
        i += 60
    
    #This is where the coodinates are averaged with the number of pixels found
    aimHoopLocation[0] = int(aimHoopLocation[0] / numOfPixelsInFirstHoop)
    aimHoopLocation[1] = int(aimHoopLocation[1] / numOfPixelsInFirstHoop) + 82
    ballHoopLocation[0] = int(ballHoopLocation[0] / numOfPixelsInSecondHoop)
    ballHoopLocation[1] = int(ballHoopLocation[1] / numOfPixelsInSecondHoop) + 82

    print("Aim hoop location: x: " + str(aimHoopLocation[0]) + " y: " + str(aimHoopLocation[1]))
    print("Ball hoop location: x: " + str(ballHoopLocation[0]) + " y: " + str(ballHoopLocation[1]))
    
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
    airTime = -yVel/accel
    
    #Velocity needed to reach point in given time
    xVel = delX / airTime
    
    #Calculate throwing angle with respect to the vertical
    throwAngle = math.atan(xVel/yVel)
    print("Angle: " + str(math.degrees(throwAngle)))
    
    #Calculate how far to pull back on screen
    yPullBack = delY / 1.7
    #yPullBack = yVel * (.2474 - (.0740 / 800) * (2872 + yVel))
    #yPullBack = yPullBack
    xPullBack = math.tan(throwAngle) * yPullBack * .8
    
    return xPullBack, yPullBack, airTime
    
    
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
    airTime = (yVelFinal - yVel) / accel
    
    #Calculate initial x velocity with time in air
    xVel = delX / airTime
    
    #Calculate throwing angle with respect to the vertical
    throwAngle = math.atan(xVel/yVel)
    print("Angle: " + str(math.degrees(throwAngle)))
    
    #Calculate how far to pull back on screen
    yPullBack = -700
    #yPullBack = -676
    xPullBack = math.tan(throwAngle) * yPullBack
    
    return xPullBack, yPullBack, airTime
    

def calcPullbackTimedThrow():
    #This function uses the normal throw method but times the throw so that it can hit a moving horizontally hoop.  It 
    # calculates the time by getting the aim hoop location and then continually doing that until the position is back 
    # to the original.

    threshold = 50 #Number of pixels used for distance from original to consider if the hoop is in the same locaion

    #Get hoop locations and time of screenshot
    startTime = time.time()  #Time of first screenshot taken
    startAimHoopLocation = findFirstPixelOccurence()  #First pixel of aim hoop given in x,y coordinates
    aimHoopLocation, ballHoopLocation = findMiddleOfHoops()  #Starting hoop locations given in x,y coordinates
    
    #Calculate distance to pull back the ball to throw it into the hoop and get the air time of ball
    xPullBack, yPullBack, airTime = calcPullbackBigThrow(aimHoopLocation, ballHoopLocation)

    #Wait a bit before checking hoop location
    time.sleep(.4)

    #Keep checking hoop locations until the hoop has made a full rotation.  This is when the hoop is heading the same 
    # direction it started
    numOfPasses = 0
    while(numOfPasses < 2):
        currentAimHoopLocation = findFirstPixelOccurence()  #First pixel of aim hoop given in x,y coordinates

        if(math.fabs(currentAimHoopLocation[0] - startAimHoopLocation[0]) > threshold):
            #Current hoop location is near the same spot as the orignal hoop location
            numOfPasses += 1

    #Get time it took to return to the original position
    returnTime = time.time()
    timeToReturnPosition = returnTime - startTime

    #Calculate the amount of time it takes from the moment of sending the command to the moment it reaches its destination
    throwTime = airTime + .5 + swipeTime + (totalCommandSendTime / numOfRuns)

    #Calculate the time to wait before throwing the ball
    waitTime = timeToReturnPosition - throwTime

    return xPullBack, yPullBack, waitTime

	
def makeShot():
    #This function puts all parts of the program together.  It figures out where the hoops are located and then
    # determines how and where to throw the ball.
    
    movementType = checkForMovingHoop()
    print("Moving: " + movementType)
    
    #Calculate the distance in pixels to pull the ball to throw it into the hoop
    if(movementType == "vertical"):
        #Throw ball really high if moving vertically
        aimHoopLocation, ballHoopLocation = findMiddleOfHoops()  #Hoop locations given in x,y coordinates
        xPullBack, yPullBack, waitTime = calcPullbackBigThrow(aimHoopLocation, ballHoopLocation)
        waitTime = 0
    elif(movementType == "horizontal"):
        #Time normal throw if moving horizontally
        xPullBack, yPullBack, waitTime = calcPullbackTimedThrow()  #Hoop locations given in x,y coordinates
    else:
        #Throw ball normally if not moving
        aimHoopLocation, ballHoopLocation = findMiddleOfHoops()
        xPullBack, yPullBack, waitTime = calcPullback(aimHoopLocation, ballHoopLocation)
        waitTime = 0
    print("xPullBack: " + str(xPullBack))
    print("yPullBack: " + str(yPullBack))
    
    #Wait before thowing if needed
    time.sleep(waitTime)

    #Issue swipe command to the phone with given pull back distances
    #device.shell("input touchscreen swipe x1 y1 x2 y2 dur")
    command = "input touchscreen swipe 700 800 " + str(700 - xPullBack) + " " + str(800 - yPullBack) + " " + str(int(swipeTime * 1000))
    print(command)
    startTime = time.time()  #Start time of command
    device.shell(command)
    finishTime = time.time()  #Time after the command was recieved and the screen was swiped
    global totalCommandSendTime
    totalCommandSendTime += (finishTime - startTime - .5)
	

numOfRuns = 0
totalCommandSendTime = 0  #This is the total time it has taken to send commands.
swipeTime = .5  #Time it takes for the phone to swipe the screen in seconds

abd = Client(host="127.0.0.1", port=5037)
devices = abd.devices()

if (len(devices) == 0):
    print("no devices")
else:
    device = devices[0] #Assume first device is the device needed
    
    for i in range(100):
        numOfRuns += 1
        print("")
        print("Shot " + str(numOfRuns))
        makeShot()
        time.sleep(4)