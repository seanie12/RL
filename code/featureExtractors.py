# featureExtractors.py
# --------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"Feature extractors for Pacman game states"

from game import Directions, Actions
import util

class FeatureExtractor:
    def getFeatures(self, state, action):
        """
          Returns a dict from features to counts
          Usually, the count will just be 1.0 for
          indicator functions.
        """
        util.raiseNotDefined()

class IdentityExtractor(FeatureExtractor):
    def getFeatures(self, state, action):
        feats = util.Counter()
        feats[(state,action)] = 1.0
        return feats

class CoordinateExtractor(FeatureExtractor):
    def getFeatures(self, state, action):
        feats = util.Counter()
        feats[state] = 1.0
        feats['x=%d' % state[0]] = 1.0
        feats['y=%d' % state[0]] = 1.0
        feats['action=%s' % action] = 1.0
        return feats

def closestFood(pos, food, walls):
    """
    closestFood -- this is similar to the function that we have
    worked on in the search project; here its all in one place
    """
    fringe = [(pos[0], pos[1], 0)]
    expanded = set()
    while fringe:
        pos_x, pos_y, dist = fringe.pop(0)
        if (pos_x, pos_y) in expanded:
            continue
        expanded.add((pos_x, pos_y))
        # if we find a food at this location then exit
        if food[pos_x][pos_y]:
            return dist
        # otherwise spread out from the location to its neighbours
        nbrs = Actions.getLegalNeighbors((pos_x, pos_y), walls)
        for nbr_x, nbr_y in nbrs:
            fringe.append((nbr_x, nbr_y, dist+1))
    # no food found
    return None

class SimpleExtractor(FeatureExtractor):
    """
    Returns simple features for a basic reflex Pacman:
    - whether food will be eaten
    - how far away the next food is
    - whether a ghost collision is imminent
    - whether a ghost is one step away
    """

    def getFeatures(self, state, action):
        # extract the grid of food and wall locations and get the ghost locations
        food = state.getFood()
        walls = state.getWalls()
        ghosts = state.getGhostPositions()
        ghostsStates = state.getGhostStates()

        features = util.Counter()

        features["bias"] = 1.0

        # compute the location of pacman after he takes the action
        x, y = state.getPacmanPosition()
        dx, dy = Actions.directionToVector(action)
        next_x, next_y = int(x + dx), int(y + dy)

        # count the number of ghosts 1-step away
        features["#-of-ghosts-1-step-away"] = sum((next_x, next_y) in Actions.getLegalNeighbors(g, walls) for g in ghosts)

        # if there is no danger of ghosts then add the food feature
        if not features["#-of-ghosts-1-step-away"] and food[next_x][next_y]:
            features["eats-food"] = 1.0

        dist = closestFood((next_x, next_y), food, walls)
        if dist is not None:
            # make the distance a number less than one otherwise the update
            # will diverge wildly
            features["closest-food"] = float(dist) / (walls.width * walls.height)
        features.divideAll(10.0)
        return features

class CustomExtractor(FeatureExtractor):
    """
    Generate your own feature
    use class AgentState in game.py, getLegalNeighbors() in game.py etc
    """
    def getFeatures(self, state, action):
        "*** YOUR CODE HERE ***"
        food = state.getFood() # Returns a Grid of boolean food indicator variables.
        walls = state.getWalls() #positions of walls
        ghosts = state.getGhostPositions() # positions of ghosts
        capsules = state.getCapsules() #positions of reamining capsules
        scaredGhosts = []
        activeGhosts = []
        features = util.Counter() #extensions of dictionary
        
        features["bias"] = 1.0
        x, y = state.getPacmanPosition()
        dx, dy = Actions.directionToVector(action)
        next_x, next_y = int(x+dx), int(y+dy)
        features["#-of-ghosts-1-step-away"] = sum( (next_x,next_y) in \
                Actions.getLegalNeighbors(g,walls) for g in ghosts)
        
        if  features["#-of-ghosts-1-step-away"] == 0 and food[next_x][next_y]:
            features["eats-food"] = 1.0
            
        dist = closestFood((next_x,next_y), food, walls)
        if dist is not None:
            features["closest-food"] = float(dist)/(walls.width*walls.height)
        #in util.py manhattanDistance( xy1, xy2 ):
        #"Returns the Manhattan distance between points xy1 and xy2"
        nearest_capsule_dist = float('inf')
        if len(capsules) == 0 :
            features["nearest-capsule-distance"] = 10
        else:
            for capsulePosition in capsules:
                distance = util.manhattanDistance([x,y],capsulePosition)
                if distance < nearest_capsule_dist:
                    nearest_capsule_dist = distance
            features["nearest-capsule-distance"] = float(nearest_capsule_dist) / (walls.width*walls.height)
        
        #ghost.scaredTimer -> time duration of scared ghost
        for ghost in state.getGhostStates():
            if ghost.scaredTimer !=0:
                scaredGhosts.append(ghost)
            else:
                activeGhosts.append(ghost)
                
        
        if len(scaredGhosts) !=0:
            dis_from_scared_ghost = min([util.manhattanDistance([x,y],ghost.getPosition()) for ghost in scaredGhosts])
            if len(activeGhosts) != 0 :    
                dis_from_active_ghost = min([util.manhattanDistance([x,y],ghost.getPosition()) for ghost in activeGhosts])
            else:
                dis_from_active_ghost = 10
            if dis_from_scared_ghost <= 3 and dis_from_active_ghost >=5:
                #features["dis-from-scared-ghost"] = dis_from_scared_ghost 
                features["#-of-ghosts-1-step-away"] = 0
                features["eats-food"] = 0
                
        
        features.divideAll(10.0)
        return features