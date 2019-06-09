#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the LunarLockout  domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

#import os for time functions
from search import * #for search engines
from lunarlockout import LunarLockoutState, Direction, lockout_goal_state #for LunarLockout specific classes and problems

#LunarLockout HEURISTICS
def heur_trivial(state):
    '''trivial admissible LunarLockout heuristic'''
    '''INPUT: a LunarLockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
    return 0

def heur_manhattan_distance(state):
    '''Manhattan distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #Write a heuristic function that uses Manhattan distance to estimate distance between the current state and the goal.
    #Your function should return a sum of the Manhattan distances between each xanadu and the escape hatch.
    distance=0
    center = int((state.width - 1) / 2)
    xanadus = state.xanadus
    print("xanadus and center position:",xanadus,center)
    if isinstance(xanadus[0],int):
        row = xanadus[0]
        col = xanadus[1]
        distance = abs(row - center) + abs(col - center)
    else:
        for x in xanadus:
            row = x[0]
            col = x[1]
            distance = distance +abs(row - center) + abs(col - center)

    return distance


def heur_L_distance(state):
    '''L distance LunarLockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    distance=0
    center = int((state.width - 1) / 2)
    xanadus = state.xanadus
    if isinstance(xanadus[0],int):
        row = xanadus[0]
        col = xanadus[1]
        if row != center :
            distance+=1
        if col != center:
            distance += 1

    else:
        for x in xanadus:
            row = x[0]
            col = x[1]
            if row != center:
                distance += 1
            if col != center:
                distance += 1

    return distance


def helper_check_block(xanadu,robort,center):
    '''helper function to check if there is a helper robort block the way of xanadu'''
    '''input: position of xanadu, robort, and center'''
    '''output: True of False'''
    block=False
    #the xanadu is at the same y axis as the center and the robort
    if xanadu[0]==robort[0]==center[0] :
        #xanadu at the top and robot block
        if xanadu[1]<robort[1]<center[1]:
            block = True
        # at the bottum and block
        elif xanadu[1] > robort[1] > center[1]:
            block = True
    # the xanadu is at the same x axis as the center and the robort
    elif xanadu[1]==robort[1]==center[1]:
        # xanadu at the right and robot block
        if xanadu[0]<robort[0]<center[0]:
            block = True
        # xanadu at the left and robot block
        elif xanadu[0]>robort[0]>center[0]:
            block = True

    return block



def heur_alternate(state):
    '''a better lunar lockout heuristic'''
    '''INPUT: a lunar lockout state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    distance=0
    center = int((state.width - 1) / 2)
    xanadus = state.xanadus
    counter=0

    for robot in state.robots:
        #check the number of robot surrands the center
        if robot == (center + 1, center) or robot == (center, center + 1) or robot == (center - 1, center) or robot == (
        center, center - 1):
            counter += 1
        # if center escape hatch is occupied by helper, distance + 1
        elif robot == (center,center):
            distance += 1
        #check if there is any robot block between xanadu and the center escape haunt.
        for xanadu in xanadus:
             block = helper_check_block(xanadu, robot, (center, center))
             if block == True:
                 distance += 2


    # center is totally surranding by helper or if center is not
    # surrounded by helper (left, right, top, button) in any 4 direction
    if counter == 4 or counter == 0:
        distance += 2
    for x in xanadus:
        counter1=0
        counter2=0
        counter3=0
        counter4=0

        for robot in state.robots:
            #if the xanadu is located at the right bottum corner. Then if there isn't a robot
            # that is outter than the xanadu it is a deadlock
            if robot[1]>x[1] and robot[0]>x[0]:
                counter1+=1
            #  xanadu is located at the left top corner. Then if there isn't a robot
            # that is outter than the xanadu it is a deadlock
            if robot[1]<x[1] and robot[0]<x[0]:
                counter2+=1
            # if the xanadu is located at the right top corner. Then if there isn't a robot
            # that is outter than the xanadu it is a deadlock
            if robot[1]<x[1] and robot[0]>x[0]:
                counter3+=1
            # xanadu is located at the left bottum corner. Then if there isn't a robot
            # that is outter than the xanadu it is a deadlock
            if robot[1]>x[1] and robot[0]<x[0]:
                counter4+=1
        #if there is any corner that contain a xanaddu that do not have a robot outter than the xanadu, it is a deadlock
        if counter1==len(state.robots) or counter2==len(state.robots) or \
                        counter3==len(state.robots) or counter4==len(state.robots):
            distance+=10000
    return distance

def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a LunarLockoutState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    fval=sN.gval+weight*sN.hval
    return fval

def anytime_weighted_astar(initial_state, heur_fn, weight=4., timebound = 2):
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a lunar lockout state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    search = SearchEngine('custom', 'full')
    time_end = os.times()[0]+timebound
    time_left = time_end - os.times()[0]
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    search.init_search(initial_state, lockout_goal_state, heur_fn, wrapped_fval_function)
    current = search.search(time_left)
    if not current:
        return False
    #if there is time left and the weight is larger than or equal to 1
    while time_left > 0 and weight >=1:
        time_left = time_end - os.times()[0]
        weight += -1
        cost=[1E10,1E10,current.gval]
        better = search.search(time_left,cost)
        if better:
            if better.gval < current.gval:
                current = better

        else:
            return current

    return current


PROBLEMS = (
  #5x5 boards: all are solveable
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((0, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 2),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 3),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((1, 4),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((2, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (0, 2),(0,4),(2,0),(4,0)),((4, 4),)), #9
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 0),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 1),)),
  LunarLockoutState("START", 0, None, 5, ((0, 0), (1, 0),(2,2),(4,2),(0,4),(4,4)),((4, 3),)),
  #7x7 BOARDS: all are solveable
  LunarLockoutState("START", 0, None, 7, ((4, 2), (1, 3), (6,3), (5,4)), ((6, 2),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (4, 2), (2,6)), ((4, 6),)),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (3, 1), (4, 1), (2,6), (4,6)), ((2, 0),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((1, 2), (0 ,2), (2 ,3), (4, 4), (2, 5)), ((2, 4),(3, 1),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 2), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((3, 1), (0 ,2), (3 ,3), (4, 4), (2, 5)), ((1, 2),(3, 0),(4, 0))),
  LunarLockoutState("START", 0, None, 7, ((2, 1), (0 ,2), (1 ,2), (6, 4), (2, 5)), ((2, 0),(3, 0),(4, 0))),
  )

if __name__ == "__main__":

  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(len(PROBLEMS)): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    print("*******RUNNING A STAR*******") 
    se = SearchEngine('astar', 'full')
    se.init_search(s0, lockout_goal_state, heur_alternate)
    final = se.search(timebound) 

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime Weighted A-star")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    weight = 4
    final = anytime_weighted_astar(s0, heur_alternate, weight, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; 
  print("Running Anytime GBFS")   

  for i in range(len(PROBLEMS)):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i]  
    final = anytime_gbfs(s0, heur_alternate, timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************")   



  

