from nbformat import current_nbformat
from numpy import number
from reverse_kin_problem import *
from time import perf_counter

class Algoritm:
    def __init__(self, problem: ReverseKinProblem, iteration_per_time=2, mintime = None):
        self.problem = problem
        self.chromosome_length = problem.robot_arm.segments_num
        self.clear_population()
        self.clear_counters()
        self.clear_logs()
        self.init_constants(iteration_per_time, mintime)
    
    def init_constants(self, iteration_per_time, mintime):
        self.iteration_per_time = iteration_per_time
        self.epsilon = 0.000001
        self.penalty = 0.5
        self.time_step = 1
        self.mintime = mintime
        self.correcting_sigma = 0.5

    # Clear functions
    def clear_logs(self):
        # used
        self.algoritm_info={}
        self.history = []
        # used - might be removed
        self.log_objective_values = None
        self.log_best_solutions = None
        self.log_best_sigmas = None

    def clear_counters(self):
        self.time = 0
        self.current_time_iteration = -1
        self.iteration = -1
        self.start_time = None

    def clear_all(self):
        self.clear_counters()
        self.clear_logs()
        self.clear_population()

    def clear_population(self):
        self.population_size = None
        self.population = None     #problem.get_random_population(self.population_size)

    # get animation info function
    def get_default_animation_info(self):
        animation_info = self.problem.get_default_animation_info()
        animation_info['title'] = f"ES, popsize={self.algoritm_info['population_size']}, offspring ={self.algoritm_info['number_of_offspring']}" # , \n $\sigma = {self.algoritm_info['sigma']}$, $tau {self.algoritm_info['tau']}$, $tau_0 {self.algoritm_info['tau0']}$"
        animation_info['text_keys'] = ["time", "iteration", "best_value", "computation_time"]
        return animation_info
    
    ###########################################
    # logging functions
    def log_history_info(self):
        current_history = {}
        current_history['time'] = self.time
        current_history['iteration'] = self.iteration
        current_history['best'] = self.best_solution
        current_history['best_value'] = self.best_solution_objective_value
        current_history['best_true_value'] = self.best_solution_true_objective_value
        current_history['bestX'] , current_history['bestY'] = self.best_solutionX, self.best_solutionY
        current_history['best_collides'] = self.best_solution_collides
        current_history['computation_time'] = perf_counter() - self.start_time
        self.history.append(current_history)

    def update_algoritm_info(self, population_size, number_of_offspring, number_of_parents, sigma, tau, tau_0):
        self.algoritm_info["population_size"] = population_size
        self.algoritm_info["number_of_offspring"] = number_of_offspring
        self.algoritm_info["number_of_parents"] = number_of_parents
        self.algoritm_info["sigma"] = sigma
        self.algoritm_info["tau"] = tau
        self.algoritm_info["tau0"] = tau_0

    def update_best_individual(self):
        if self.best_solution_objective_value > self.current_population_objective_values[0]:
            self.best_solution= self.current_population_solutions[0, :]
            self.best_solution_objective_value = self.current_population_objective_values[0]
            self.best_solution_true_objective_value = self.current_population_true_objective_values[0]
            self.best_solutionX, self.best_solutionY = self.X[0], self.Y[0]
            # set if collides:
            self.best_solution_collides = self.current_population_colliding_idx[0]
    def log_stats(self):
        self.log_objective_values[self.iteration, :] = [self.current_population_objective_values.min(), self.current_population_objective_values.max(), self.current_population_objective_values.mean(), self.current_population_objective_values.std()]
        self.log_best_solutions[self.iteration, :] = self.current_population_solutions[0, :]
        self.log_best_sigmas[self.iteration, :] = self.current_population_sigmas[0, :]
    
    # Termination condition:
    def termination_condition(self, number_of_iterations):
        self.iteration +=1
        self.current_time_iteration +=1
        if self.iteration<number_of_iterations:
            if self.current_time_iteration>= self.iteration_per_time:
                self._increment_time()
            # jeśli znajdziemy wystarczająco dobre rozwiązanie to od razu zwiększamy czas
            elif not self.best_solution_collides and abs(self.best_solution_objective_value)<self.epsilon:
                self._increment_time()
            return True
        else: 
            return False

    def _increment_time(self):
        self.time += self.time_step
        self.current_time_iteration = -1
        self.problem.obstacles.move(self.time)
        self._check_collision_current_population()
        self._check_collision_best_individual()

    def _check_collision_best_individual(self):
        if self.problem.obstacles.intersects(LineString(zip(self.best_solutionX, self.best_solutionY))):
            self._clear_best_solution()


    #######################################
    # Evolution strategy helper functions
    def _clear_best_solution(self):
        self.best_solution= np.empty((1, self.chromosome_length))
        self.best_solution_objective_value = np.inf
        self.best_solution_true_objective_value = np.inf
        self.best_solutionX = None
        self.best_solutionY = None
        self.best_solution_collides = None

    def _create_initial_population(self, population_size, sigma):
        self.current_population_solutions = self.problem.get_random_population(population_size)
        self.current_population_sigmas =  sigma * np.ones((population_size, self.chromosome_length))
        self.current_population_true_objective_values, self.X, self.Y = self.problem.evaluate_population(self.current_population_solutions)
        self.current_population_objective_values = np.copy(self.current_population_true_objective_values)
        self._check_collision_current_population()

    def _check_collision_current_population(self, sigma = 1):
        # compute collisions for population
        self.current_population_colliding_idx = self.problem.get_colliding_idx(self.X, self.Y)
        ## ver1: decrease objective values of incorrect solutions:
        self.current_population_objective_values += self.current_population_colliding_idx*self.penalty
        ## ver2: increase sigmas of incorrect solutions by correcting_sigma
        add_sigma_shape = self.current_population_sigmas[self.current_population_colliding_idx, :].shape
        self.current_population_sigmas[self.current_population_colliding_idx, :] = self.correcting_sigma * np.ones(add_sigma_shape)

    def _init_logging_arrays(self, number_of_iterations):
        self.log_objective_values = np.empty((number_of_iterations, 4))
        self.log_best_solutions = np.empty((number_of_iterations, self.chromosome_length))
        self.log_best_sigmas = np.empty((number_of_iterations, self.chromosome_length))
    
    # selecting the parent indices by the roulette wheel method
    def _select_parent_indices(self, population_size, number_of_offspring, number_of_parents):
        self.fitness_values = self.current_population_objective_values.max() - self.current_population_objective_values
        if self.fitness_values.sum() > 0:
            self.fitness_values = self.fitness_values / self.fitness_values.sum()
        else:
            self.fitness_values = 1.0 / population_size * np.ones(population_size)
        self.parent_indices = np.random.choice(population_size, (number_of_offspring, number_of_parents), True, self.fitness_values).astype(np.int64)
    
    # creating the children population by Global Intermediere Recombination
    def _create_children_population(self, number_of_offspring):
        self.children_population_solutions = np.zeros((number_of_offspring, self.chromosome_length))
        self.children_population_sigmas = np.zeros((number_of_offspring, self.chromosome_length))
        for i in range(number_of_offspring):
            self.children_population_solutions[i, :] = self.current_population_solutions[self.parent_indices[i, :], :].mean(axis=0)
            self.children_population_sigmas[i, :] = self.current_population_sigmas[self.parent_indices[i, :], :].mean(axis=0)

    def _mutate_children(self, number_of_offspring, tau, tau_0):
        # mutating the children population by adding random gaussian noise
        self.children_population_sigmas = self.children_population_sigmas * np.exp(tau * np.random.randn(number_of_offspring, self.chromosome_length) + tau_0 * np.random.randn(number_of_offspring, 1))
        self.children_population_solutions = self.children_population_solutions + self.children_population_sigmas * np.random.randn(number_of_offspring, self.chromosome_length)
        # correcting children to match arm restrictions
        self.children_population_solutions = self.problem.correct_population(self.children_population_solutions)

    def _evaluate_children(self):
        # evaluating the objective function on the children population
        self.children_population_true_objective_values, self.childX, self.childY = self.problem.evaluate_population(self.children_population_solutions)
        self.children_population_objective_values = np.copy(self.children_population_true_objective_values)
        # evaluate correctness of children population
        self.children_population_colliding_idx = self.problem.get_colliding_idx(self.childX, self.childY)
        # ver1: decrease objective values of incorrect solutions (by 0.5)
        self.children_population_objective_values += self.children_population_colliding_idx*self.penalty
        
    # replacing the current population by (Mu + Lambda) Replacement
    def _mu_plus_lambda_replacement(self):
        self.X = np.vstack([self.X, self.childX])
        self.Y = np.vstack([self.Y, self.childY])
        self.current_population_objective_values = np.hstack([self.current_population_objective_values, self.children_population_objective_values])
        self.current_population_true_objective_values = np.hstack([self.current_population_true_objective_values, self.children_population_true_objective_values])
        self.current_population_solutions = np.vstack([self.current_population_solutions, self.children_population_solutions])
        self.current_population_sigmas = np.vstack([self.current_population_sigmas, self.children_population_sigmas])
        self.current_population_colliding_idx = np.hstack([self.current_population_colliding_idx, self.children_population_colliding_idx])

    def _crop_population(self, population_size):
        I = np.argsort(self.current_population_objective_values)#[::-1]
        self.current_population_solutions = self.current_population_solutions[I[:population_size], :]
        self.current_population_sigmas = self.current_population_sigmas[I[:population_size], :]
        self.current_population_objective_values = self.current_population_objective_values[I[:population_size]]
        self.current_population_true_objective_values = self.current_population_true_objective_values[I[:population_size]]
        self.X = self.X[I[:population_size], :]
        self.Y = self.Y[I[:population_size], :]
        self.current_population_colliding_idx = self.current_population_colliding_idx[I[:population_size]]

    def es_algoritm_setup(self, number_of_iterations, population_size, number_of_offspring, number_of_parents, sigma, tau, tau_0):
        if self.mintime:
            iters = self.mintime*self.iteration_per_time
            if number_of_iterations<iters:
                print("corrected number of iterations to match requested time")
                number_of_iterations = iters
        self.update_algoritm_info(population_size, number_of_offspring, number_of_parents, sigma, tau, tau_0)  
        self._clear_best_solution()
        self._init_logging_arrays(number_of_iterations)
        

    ###############################
    # Evolution strategy algoritm #
    ###############################
    def es(self, number_of_iterations, population_size, number_of_offspring, number_of_parents, sigma, tau, tau_0):
        logging_dist = np.ceil(number_of_iterations/10)
        self.start_time = perf_counter()
        # algoritm setup
        self.es_algoritm_setup(number_of_iterations, population_size, number_of_offspring, number_of_parents, sigma, tau, tau_0)

        # create initial population and evaluate it's score and correctess
        self._create_initial_population(population_size, sigma)

        while(self.termination_condition(number_of_iterations)):
            self._select_parent_indices(population_size, number_of_offspring, number_of_parents)
            self._create_children_population(number_of_offspring)

            self._mutate_children(number_of_offspring, tau, tau_0) # mutate and crop to constraints
            self._evaluate_children()

            self._mu_plus_lambda_replacement()
            self._crop_population(population_size)

            self.update_best_individual()
            self.log_stats()
            self.log_history_info()

            if np.mod(self.iteration, logging_dist) == 0:
                print("Iteration %04d : best score = %0.8f, mean score = %0.8f." % (self.iteration, self.best_solution_objective_value, self.log_objective_values[self.iteration, 2]))

        return self.best_solution_objective_value, self.best_solution, self.log_objective_values, self.log_best_solutions, self.log_best_sigmas