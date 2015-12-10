# File created by Thibaut Royer, Epitech school
# thibaut1.royer@epitech.eu
# It intends to be an example program for the "Two wheels, one arm" educative project.

import vrep
import math
import random
import time
import logging

logging.basicConfig(filename='2w1a.log',level=logging.DEBUG)
#logging.info('>>>logging start<<<')

def fitness(lop, rop, lp, rp):
    ldis = math.sqrt((lop[0] - lp[0])*(lop[0] - lp[0]) + (lop[1] - lp[1])*(lop[1] - lp[1]))
    rdis = math.sqrt((rop[0] - rp[0])*(rop[0] - rp[0]) + (rop[1] - rp[1])*(rop[1] - rp[1]))
    rs = (ldis + rdis) / (abs(ldis - rdis)/min(ldis, rdis))
    msg = "The distance moved of Left and Right wheels: " + str(ldis) + "  " + str(rdis) + "\n The fitness = " + str(rs)
    print msg
    logging.info(msg)
    return rs

# evolution individual is [awrist, aelbow, ashoulder, dis]
def evolve(pop, retain=0.2, random_select=0.05, mutate=0.01):
    pop = sorted(pop, key = lambda x:x[3])
    pop.reverse()
    print "The population after sorted: " + str(pop)
    logging.info("The population after sorted: " + str(pop))
    retain_length = int(len(pop)*retain)
    parents = pop[:retain_length]
    # randomly add other individuals to promote genetic diversity
    for individual in pop[retain_length:]:
        if random_select > random.random():
            parents.append(individual)
    # mutate some individuals
    for individual in parents:
        if mutate > random.random():
            pos_to_mutate = random.randint(0, len(individual)-2)
            individual[pos_to_mutate] = random.randint(0, 300)
    # crossover parents to create children
    parents_length = len(parents)
    desired_length = len(pop) - parents_length
    children = []
    while len(children) < desired_length:
        male = random.randint(0, parents_length-1)
        female = random.randint(0, parents_length-1)
        if male != female:
            male = parents[male]
            female = parents[female]
            if random.randint(0, 1) == 0:
                child = male[:1] + female[1:]
                msg = "0. M: " + str(male) + "  F: " + str(female) + "  Child: " + str(child)
                print msg
                logging.info(msg)
            else:
                child = male[:2] + female[2:]
                msg = "1. M: " + str(male) + "  F: " + str(female) + "  Child: " + str(child)
                print msg
                logging.info(msg)
            children.append(child)
    parents.extend(children)
    print "The population after evolution: " + str(pop)
    logging.info("/n>>>The population after evolution: <<</n" + str(pop))
    return parents


print ('Start')

# Close eventual old connections
vrep.simxFinish(-1)
# Connect to V-REP remote server
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print ('Connected to remote API server')

    # Communication operating mode with the remote API : wait for its answer before continuing (blocking mode)
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
    opmode = vrep.simx_opmode_oneshot_wait

    # Try to retrieve motors and robot handlers
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectHandle
    ret1, wristHandle = vrep.simxGetObjectHandle(clientID, "WristMotor", opmode)
    ret2, elbowHandle = vrep.simxGetObjectHandle(clientID, "ElbowMotor", opmode)
    ret3, shoulderHandle = vrep.simxGetObjectHandle(clientID, "ShoulderMotor", opmode)
    ret4, leftWheelHandle = vrep.simxGetObjectHandle(clientID, "LeftWheelJoint", opmode)
    ret5, rightWheelHandle = vrep.simxGetObjectHandle(clientID, "RightWheelJoint", opmode)
    ret6, robotHandle = vrep.simxGetObjectHandle(clientID, "2W1A", opmode)

    # If handlers are OK, execute three random simulations
    if ret1 == 0 and ret2 == 0 and ret3 == 0 and ret4 == 0 and ret5 == 0 and ret6 == 0:
        random.seed()
        for i in range(0, 3):
            # Start the simulation
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStartSimulation
            vrep.simxStartSimulation(clientID, opmode)
            time.sleep(3)
            msg = "----- Simulation started -----"
            print msg
            logging.info(msg)

            # Start getting the robot position
            # Unlike other commands, we will use a streaming operating mode
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectPosition
            pret, robotPos = vrep.simxGetObjectPosition(clientID, robotHandle, -1, vrep.simx_opmode_streaming)
            msg = "2w1a position: (x = " + str(robotPos[0]) +\
                  ", y = " + str(robotPos[1]) + ") res = " + str(pret)
            print msg
            logging.info(msg)

            # Start getting the robot orientation
            # Unlike other commands, we will use a streaming operating mode
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetObjectOrientation
            oret, robotOrient = vrep.simxGetObjectOrientation(clientID, robotHandle, -1, vrep.simx_opmode_streaming)
            msg =  "2w1a orientation: (x = " + str(robotOrient[0]) + \
                  ", y = " + str(robotOrient[1]) +\
                  ", z = " + str(robotOrient[2]) + ")"
            print msg
            logging.info(msg)

            # define the population for evolution
            population = []
            
            # Make the robot move randomly
            for j in range(0, 100):
                vrep.simxStartSimulation(clientID, opmode)
                # Generating random positions for the motors
                awrist = random.randint(0, 300)
                aelbow = random.randint(0, 300)
                ashoulder = random.randint(0, 300)

                # Get the robot position before the movement sequence
                pret, lOriginPos = vrep.simxGetObjectPosition(clientID, leftWheelHandle, -1, vrep.simx_opmode_streaming)
                pret, rOriginPos = vrep.simxGetObjectPosition(clientID, rightWheelHandle, -1, vrep.simx_opmode_streaming)
                msg = "2w1a origin position: leftWheel = " + str(lOriginPos) +\
                      ", rightWheel = " + str(rOriginPos) + ")"
                print msg
                logging.info(msg)
                
                # The control functions use Radians to determine the target position.
                # Here, we use maths.radians to convert degrees into radians.
                # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxSetJointTargetPosition
                msg = "Motors target positions: " + str(ashoulder) + " " + str(aelbow) + " " + str(awrist)
                print msg
                logging.info(msg)
                vrep.simxSetJointTargetPosition(clientID, wristHandle, math.radians(awrist), opmode)
                vrep.simxSetJointTargetPosition(clientID, elbowHandle, math.radians(aelbow), opmode)
                vrep.simxSetJointTargetPosition(clientID, shoulderHandle, math.radians(ashoulder), opmode)

                # Wait in order to let the motors finish their movements
                # Tip: there must be a more efficient way to do it...
                time.sleep(3)

                # Get the motors effective positions after the movement sequence
                # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetJointPosition
                pwrist = vrep.simxGetJointPosition(clientID, wristHandle, opmode)
                pelbow = vrep.simxGetJointPosition(clientID, elbowHandle, opmode)
                pshoulder = vrep.simxGetJointPosition(clientID, shoulderHandle, opmode)
                msg = "Motors reached positions: " + str(pshoulder) + " " + str(pelbow) + " " + str(pwrist)
                #print msg

                # Get the robot position after the movement sequence
                pret, lPos = vrep.simxGetObjectPosition(clientID, leftWheelHandle, -1, vrep.simx_opmode_streaming)
                pret, rPos = vrep.simxGetObjectPosition(clientID, rightWheelHandle, -1, vrep.simx_opmode_streaming)
                msg = "2w1a position after move: leftWheel = " + str(lPos) +\
                      ", rightWheel = " + str(rPos) + ")"
                print msg
                logging.info(msg)

                # find out the fitness robot moved
                rs = fitness(lOriginPos, rOriginPos, lPos, rPos)
                msg = "The fitness Motors moved is: " + str(rs)
                #print msg
                #logging.info(msg)

                population.append([awrist, aelbow, ashoulder, rs])
                #print "The evolution individual is [awrist, aelbow, ashoulder, dis]: \n" + str(population)

                # Get the robot orientation after the movement sequence
                oret, robotOrient = vrep.simxGetObjectOrientation(clientID, robotHandle, -1, vrep.simx_opmode_buffer)
                msg = "2w1a orientation: (x = " + str(robotOrient[0]) +\
                      ", y = " + str(robotOrient[1]) +\
                      ", z = " + str(robotOrient[2]) + ")"
                print msg
                logging.info(msg)

                msg = "\n----- next move -----\n"
                print msg
                logging.info(msg)
                
                vrep.simxStopSimulation(clientID, opmode)
                time.sleep(1)

            print "The evolution individual is [awrist, aelbow, ashoulder, dis]: \n" + str(population)
            # evolution for 20 times
            for i in xrange(10):
                
                msg = "\n>>>The " + str(i) + " generations: <<<\n"
                print msg
                logging.info(msg)
                population = evolve(population)
                
                # get the distance again
                for j in range(0, len(population)-1):
                    vrep.simxStartSimulation(clientID, opmode)
                    awrist = population[j][0]
                    aelbow = population[j][1]
                    ashoulder = population[j][3]

                    # Get the robot position before the movement sequence
                    pret, lOriginPos = vrep.simxGetObjectPosition(clientID, leftWheelHandle, -1, vrep.simx_opmode_streaming)
                    pret, rOriginPos = vrep.simxGetObjectPosition(clientID, rightWheelHandle, -1, vrep.simx_opmode_streaming)
                    msg = "2w1a origin position: leftWheel = " + str(lOriginPos) +\
                          ", rightWheel = " + str(rOriginPos) + ")"
                    #print msg
                    
                    # The control functions use Radians to determine the target position.
                    # Here, we use maths.radians to convert degrees into radians.
                    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxSetJointTargetPosition
                    msg = "Motors target positions: " + str(ashoulder) + " " + str(aelbow) + " " + str(awrist)
                    print msg
                    logging.info(msg)
                    vrep.simxSetJointTargetPosition(clientID, wristHandle, math.radians(awrist), opmode)
                    vrep.simxSetJointTargetPosition(clientID, elbowHandle, math.radians(aelbow), opmode)
                    vrep.simxSetJointTargetPosition(clientID, shoulderHandle, math.radians(ashoulder), opmode)

                    # Wait in order to let the motors finish their movements
                    # Tip: there must be a more efficient way to do it...
                    time.sleep(3)

                    # Get the motors effective positions after the movement sequence
                    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxGetJointPosition
                    #pwrist = vrep.simxGetJointPosition(clientID, wristHandle, opmode)
                    #pelbow = vrep.simxGetJointPosition(clientID, elbowHandle, opmode)
                    #pshoulder = vrep.simxGetJointPosition(clientID, shoulderHandle, opmode)
                    #msg = "Motors reached positions: " + str(pshoulder) + " " + str(pelbow) + " " + str(pwrist)
                    #print msg

                    # Get the robot position after the movement sequence
                    pret, lPos = vrep.simxGetObjectPosition(clientID, leftWheelHandle, -1, vrep.simx_opmode_streaming)
                    pret, rPos = vrep.simxGetObjectPosition(clientID, rightWheelHandle, -1, vrep.simx_opmode_streaming)
                    msg = "2w1a position after move: leftWheel = " + str(lPos) +\
                          ", rightWheel = " + str(rPos) + ")"
                    #print msg
                    #logging.info(msg)

                    # find out the fitness robot moved
                    rs = fitness(lOriginPos, rOriginPos, lPos, rPos)
                    msg = "The fitness Motors moved is: " + str(rs)
                    #print msg
                    #logging.info(msg)

                    population[j][3] = rs
                    msg = "The evolution individual is [awrist, aelbow, ashoulder, dis]: \n" + str(population[j])
                    print msg
                    logging.info(msg)

                    # Get the robot orientation after the movement sequence
                    #oret, robotOrient = vrep.simxGetObjectOrientation(clientID, robotHandle, -1, vrep.simx_opmode_buffer)
                    #msg = "2w1a orientation: (x = " + str(robotOrient[0]) +\
                    #      ", y = " + str(robotOrient[1]) +\
                    #      ", z = " + str(robotOrient[2]) + ")"
                    #print msg
                    
                    vrep.simxStopSimulation(clientID, opmode)
                    time.sleep(1)

            # End the simulation, wait to be sure V-REP had the time to stop it entirely
            # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxStopSimulation
            vrep.simxStopSimulation(clientID, opmode)
            time.sleep(1)
            msg = "----- Simulation ended -----"
            print msg
            logging.info(msg)

    # Close the connection to V-REP remote server
    # http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm#simxFinish
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('End')
