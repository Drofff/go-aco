package aco

import "fmt"

type Configuration struct {
	// NumOfAnts defines the number of ants to be produced within a single iteration.
	NumOfAnts int
	// NumOfIterations defines the number of generations of ants that will be produced. Pheromone values are updated
	// after every iteration i.e. stay the same during an iteration.
	NumOfIterations int
	// PheromoneImportanceWeight defines how much the pheromone amount on the path influences the decision to take the path.
	PheromoneImportanceWeight float64
	// VisibilityImportanceWeight defines how much the visibility influences the ant's decision to take a specific path.
	VisibilityImportanceWeight float64
	// EvaporationRate defines the % of pheromone that evaporates after each iteration.
	EvaporationRate float64
	// PheromoneDepositStrength defines the maximum amount of pheromone that can be deposited at a time.
	//
	// The amount of pheromone deposited by an ant (dp) is defined as:
	//  dp = pds / rc, where pds is the strength value, rc is the distance (cost) of the route the ant has taken.
	PheromoneDepositStrength float64
	// MaxSearchDepth limits the number of edges (paths) one ant can travel through.
	MaxSearchDepth int
}

func (c Configuration) validate() error {
	if c.NumOfAnts <= 0 {
		return fmt.Errorf("NumOfAnts must be greater than 0")
	}
	if c.NumOfIterations <= 0 {
		return fmt.Errorf("NumOfIterations must be greater than 0")
	}
	if c.EvaporationRate < 0.001 || c.EvaporationRate > 0.999 {
		return fmt.Errorf("EvaporationRate must be between 0.001 and 0.999")
	}
	if c.PheromoneDepositStrength <= 0 {
		return fmt.Errorf("PheromoneDepositStrength must be greater than 0")
	}
	if c.MaxSearchDepth <= 0 {
		return fmt.Errorf("MaxSearchDepth must be greater than 0")
	}
	return nil
}
