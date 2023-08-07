import numpy

'''
formula: Output = Kp * e(n) + Ki * sum(e(1)+...+e(n)) + Kd * (e(n) - e(n-1))
e = Output - SetPoint
'''
def boundary(b_min, b_max, value):
    if value < b_min:
        value = b_min
    elif value > b_max:
        value = b_max

    return value

def checkboundary(b_min, b_max, a):
    ran = len(a)
    best = numpy.zeros(ran)
    for i in range(0, ran):
        best[i] = boundary(b_min, b_max, a[i])

    return best

def cal_pop_fitness(errors, pop, setPoint):
    s = numpy.sum(pop * errors, axis=1)
    points = numpy.full(shape=s.shape, fill_value=setPoint)
    fitness = abs(setPoint - s)
    return fitness

def select_mating_pool(pop, fitness, num_parents, b_min, b_max):
    # Selecting the best individuals in the current generation as parents for producing the offspring of the next generation.
    parents = numpy.empty((num_parents, pop.shape[1]))
    for parent_num in range(num_parents):
        min_fitness_idx = numpy.where(fitness == numpy.min(fitness))
        min_fitness_idx = min_fitness_idx[0][0]

        # ran = len(pop[min_fitness_idx, :])
        # best = numpy.zeros(ran)
        # for i in range(0, ran):
        #     best[i] = checkboundary(b_min, b_max, pop[min_fitness_idx, i])     
        best = checkboundary(b_min, b_max, pop[min_fitness_idx, :])

        # parents[parent_num, :] = pop[min_fitness_idx, :]
        parents[parent_num, :] = best
        fitness[min_fitness_idx] = -99999999999
    return parents

def crossover(parents, offspring_size):
    offspring = numpy.empty(offspring_size)
    # The point at which crossover takes place between two parents. Usually, it is at the center.
    crossover_point = numpy.uint8(offspring_size[1]/2)

    for k in range(offspring_size[0]):
        # Index of the first parent to mate.
        parent1_idx = k%parents.shape[0]
        # Index of the second parent to mate.
        parent2_idx = (k+1)%parents.shape[0]
        # The new offspring will have its first half of its genes taken from the first parent.
        offspring[k, 0:crossover_point] = parents[parent1_idx, 0:crossover_point]
        # The new offspring will have its second half of its genes taken from the second parent.
        offspring[k, crossover_point:] = parents[parent2_idx, crossover_point:]
    return offspring

def mutation(offspring_crossover, b_min: float, b_max: float, num_mutations=1):
    mutations_counter = numpy.uint8(offspring_crossover.shape[1] / num_mutations)
    # Mutation changes a number of genes as defined by the num_mutations argument. The changes are random.
    for idx in range(offspring_crossover.shape[0]):
        gene_idx = mutations_counter - 1
        for mutation_num in range(num_mutations):
            # The random value to be added to the gene.
            random_value = numpy.random.uniform(-1.0, 1.0, 1)
            value = offspring_crossover[idx, gene_idx] + random_value
            best = checkboundary(b_min, b_max, value)
            offspring_crossover[idx, gene_idx] = best
            gene_idx = gene_idx + mutations_counter
    return offspring_crossover