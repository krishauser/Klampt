"""Stores common semantic properties for robots."""

props = {'hubo-II+':{'freeBase':True,
                     'numLegs':2,
                     'feet':{'l':56,'r':62},
                     'legsAndBase':[0,1,2,3,4,5,50,51,52,53,54,55,56,57,58,59,60,61,62],
                     'lowerBody':[50,51,52,53,54,55,56,57,58,59,60,61,62],
                     'legs':{'l':[51,52,53,54,55,56],'r':[57,58,59,60,61,62]},
                     'numArms':2,
                     'hands':[13,34],
                     'arms':{'r':range(29,35),'l':range(8,14)}
                     },
         'drchubo-v3':{'freeBase':True,
                       'numLegs':2,
                       'feet':{'l':52,'r':59},
                       'legsAndBase':[0,1,2,3,4,5,46,47,48,49,50,51,52,54,55,56,57,58,59],
                       'lowerBody':[46,47,48,49,50,51,52,54,55,56,57,58,59],
                       'legs':{'l':[47,48,49,50,51,52],'r':[54,55,56,57,58,59]},
                       'numArms':2,
                       'hands':[12,32],
                       'arms':{'l':range(6,13),'r':range(26,33)}
                       }
         }
                 

def freeBase(robot):
    return props[robot.getName()]['freeBase']

def numLegs(robot):
    return props[robot.getName()]['numLegs']

def feet(robot):
    return props[robot.getName()]['feet']

def legsAndBase(robot):
    return props[robot.getName()]['legsAndBase']

def lowerBody(robot):
    return props[robot.getName()]['lowerBody']

def legs(robot):
    return props[robot.getName()]['legs']

def leg(robot,index):
    return props[robot.getName()]['legs'][index]

def numArms(robot):
    return props[robot.getName()]['numArms']

def hands(robot):
    return props[robot.getName()]['hands']

def arms(robot,index):
    return props[robot.getName()]['arms']

def arm(robot,index):
    return props[robot.getName()]['arms'][index]
