import pickle
import numpy as np, random, operator, pandas as pd, matplotlib.pyplot as plt
import cv2
from PIL import Image
from math import sqrt

class City:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def distance(self,city):
       
        # fromCity = (self.x,self.y)
        # toCity = (city.x,city.y)
        # cost = costs[fromCity][toCity]

        cost = sqrt((self.x - city.x)**2 + (self.y - city.y)**2)

        return cost
    def __repr__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

class Fitness:
    def __init__ (self,route):
        self.route = route
        self.distance = 0
        self.fitness = 0.0

    def routeDistance(self):
        if self.distance == 0:
            pathDistance = 0
            for i in range(0,len(self.route)):
                fromCity = self.route[i]
                toCity = None
                if i + 1 < len(self.route):
                    toCity = self.route[i+1]
                else:
                     toCity = self.route[0]
                pathDistance += fromCity.distance(toCity)
            self.distance = pathDistance
        return self.distance

    def routeFitness(self):
        if self.fitness == 0:
            self.fitness = 1/float(self.routeDistance())

        return self.fitness

def createRoute(cityList):
    route = random.sample(cityList,len(cityList))

    return route


def initialPopulation (popSize, cityList):
    population = []

    for i in range(0,popSize):
        population.append(createRoute(cityList))

    return population


def rankRoutes(population):
    fitnessResults = {}
    for i in range(0,len(population)):
        fitnessResults[i] = Fitness(population[i]).routeFitness()
    return sorted(fitnessResults.items(), key = operator.itemgetter(1), reverse = True)


def selection(popRanked, eliteSize):
    selectionResults = []
    df = pd.DataFrame(np.array(popRanked), columns = ["Index", "Fitness"])
    df['cum_sum'] = df.Fitness.cumsum()
    df['cum_perc'] = 100*df.cum_sum/df.Fitness.sum()

    for i in range(0,eliteSize):
        selectionResults.append(popRanked[i][0])
    for i in range(0, len(popRanked) - eliteSize):
        pick = 100*random.random()
        for i in range(0, len(popRanked)):
            if pick <= df.iat[i,3]:
                selectionResults.append(popRanked[i][0])
                break

    return selectionResults


def matingPool(population, selectionResults):
    matingpool = []
    for i in range(0,len(selectionResults)):
        index = selectionResults[i]
        matingpool.append(population[index])
    return matingpool


def breed(parent1,parent2):
    child = []
    childP1 = []
    childP2 = []

    geneA = int(random.random() * len(parent1))
    geneB = int(random.random() * len(parent1))

    startGene = min(geneA, geneB)
    endGene = max(geneA, geneB)

    for i in range(startGene, endGene):
        childP1.append(parent1[i])

    childP2 = [item for item in parent2 if item not in childP1]

    child = childP1 + childP2

    return child


def breedPopulation(matingpool, eliteSize):
    children = []
    length = len(matingpool) - eliteSize
    pool = random.sample(matingpool, len(matingpool))

    for i in range(0,eliteSize):
        children.append(matingpool[i])

    for i in range(0,length):
        child = breed(pool[i], pool[len(matingpool)-i-1])
        children.append(child)

    return children


def mutate(individual, mutationRate):
    for swapped in range(len(individual)):
        if(random.random() < mutationRate):
            swapWith = int(random.random() * len(individual))

            city1 = individual[swapped]
            city2 = individual[swapWith]

            individual[swapped] = city2
            individual[swapWith] = city1
    return individual


def mutatePopulation(population, mutationRate):
    mutatedPop = []

    for ind in range(0, len(population)):
        mutatedInd = mutate(population[ind], mutationRate)
        mutatedPop.append(mutatedInd)

    return mutatedPop


def nextGeneration(currentGen, eliteSize, mutationRate):
    popRanked = rankRoutes(currentGen)
    selectionResults = selection(popRanked, eliteSize)
    matingpool = matingPool(currentGen, selectionResults)
    children = breedPopulation(matingpool,eliteSize)
    nextGeneration = mutatePopulation(children, mutationRate)

    return nextGeneration


def geneticAlgorithm (coordinates, popSize, eliteSize, mutationRate, generations, image, visualise = True):
    
    df = pd.DataFrame(coordinates,columns = ['X','Y'])
    pd.set_option("display.max_rows", None, "display.max_columns", None)

    population =[]
    for i in range(0,len(coordinates)):
        population.append(City(x=df.iat[i,0], y=df.iat[i,1]))
    
    pop = initialPopulation(popSize, population)
    print("Initial distance: " + str(1/rankRoutes(pop)[0][1]))

    for i in range (0,generations):
        pop = nextGeneration(pop, eliteSize, mutationRate)

    print("Final distance: " + str(1/rankRoutes(pop)[0][1]))
    bestRouteIndex = rankRoutes(pop)[0][0]
    bestRoute = pop[bestRouteIndex]
    
    bestRoute = [(city.x,city.y) for city in bestRoute]
    # Rearrange index so bestRoute starts at green coordinate
    start = bestRoute.index((511,403))
    bestRoute = bestRoute[start:] + bestRoute[:start]

    if visualise == True:
        img = cv2.imread(image)
        for i in range(0,len(bestRoute)):
            if i+1 == len(bestRoute):
                p1 = (bestRoute[i][0],bestRoute[i][1])
                p2 = (bestRoute[0][0],bestRoute[0][1])
            else:
                p1 = (bestRoute[i][0],bestRoute[i][1])
                p2 = (bestRoute[i+1][0],bestRoute[i+1][1])

            cv2.line(img,p1,p2,(255,0,0),5)
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        # cv2.imshow("image",img)
        # cv2.waitKey(0)
        Image.fromarray(img).show()

    return bestRoute


if __name__ == "__main__":
    
    with open("Coordinates.pkl","rb") as f:
        coord =pickle.load(f)
    f.close()

    # with open("PathPlan/OutputFiles/Costs.pkl","rb") as f:
    #     costs =pickle.load(f)
    # f.close()

    coord_mod = np.vstack((coord['green'],coord['red']))

    bestRoute = geneticAlgorithm(coordinates = coord_mod, popSize = 100, eliteSize = 10, mutationRate = 0.01, generations = 500, image = "MapImg.png", visualise = True)
    print(bestRoute)

    