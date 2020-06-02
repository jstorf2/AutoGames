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
    print(math.degrees(throwAngle))
    
    #Calculate how far to pull back on screen
    yPullBack = -700
    #yPullBack = -676
    xPullBack = math.tan(throwAngle) * yPullBack
    
    return xPullBack, yPullBack