import numpy 
import gaPID
import math
import time


'''
formula: Output = Kp * e(n) + Ki * sum(e(1)+...+e(n)) + Kd * (e(n) - e(n-1))
e = Output - SetPoint
'''
true_Kp, true_Ki, true_Kd = 2.1, 0.01, 0.15
en, esum, ed = 1.0, 1.0, 1.0
errors = [en, esum, ed]
setPoint = 0.0

num_weights = len(errors)
sol_per_pop = 8
num_parents_mating = 4

b_min, b_max = 0, 10.0
pop_size = (sol_per_pop, num_weights)
new_population = numpy.random.uniform(low=b_min, high=b_max, size= pop_size)

best_outputs = []
num_generations = 20

def getTrueOutput(rad):
    # output = math.cos(rad) * math.sin(rad) + math.cos(rad) + math.sin(rad)
    output = math.cos(rad) * math.sin(rad)
    return output

def update(t):
    global en, esum, ed, setPoint
    output = getTrueOutput(t/360.0)
    en1 = en
    en = setPoint - output
    esum += en
    ed = en - en1
    setPoint = output
    print('update:', en, esum, ed)

for generation in range(num_generations):
    t = time.time()
    print("Generation : ", generation)
    update(generation)

    # Measuring the fitness of each chromosome in the population.
    fitness = gaPID.cal_pop_fitness([en, esum, ed], new_population, setPoint)
    print("Fitness")
    print(fitness)    

    min_outputs = numpy.min(fitness)
    # best_match_idx = numpy.where(fitness == min_outputs)    
    best_outputs.append(min_outputs)
    print("Best result : ", min_outputs)
    
    # Selecting the best parents in the population for mating.
    parents = gaPID.select_mating_pool(new_population, fitness, 
                                      num_parents_mating, b_min, b_max)
    print("Parents")
    print(parents)

    # Generating next generation using crossover.
    offspring_crossover = gaPID.crossover(parents,
                                       offspring_size=(pop_size[0]-parents.shape[0], num_weights))
    print("Crossover")
    print(offspring_crossover)

    # Adding some variations to the offspring using mutation.
    offspring_mutation = gaPID.mutation(offspring_crossover, b_min, b_max, num_mutations=2)
    print("Mutation")
    print(offspring_mutation)

    # Creating the new population based on the parents and offspring.
    new_population[0:parents.shape[0], :] = parents
    new_population[parents.shape[0]:, :] = offspring_mutation

    print(f'total time: {time.time() - t}')

update(num_generations)
# Getting the best solution after iterating finishing all generations.
#At first, the fitness is calculated for each solution in the final generation.
fitness = gaPID.cal_pop_fitness([en, esum, ed], new_population, setPoint)
# Then return the index of that solution corresponding to the best fitness.
best_match_idx = numpy.where(fitness == numpy.min(fitness))

print("Best solution : ", new_population[best_match_idx, :])
print("Best solution fitness : ", fitness[best_match_idx])

import matplotlib.pyplot
matplotlib.pyplot.plot(best_outputs)
matplotlib.pyplot.xlabel("Iteration")
matplotlib.pyplot.ylabel("Fitness")
matplotlib.pyplot.show()




