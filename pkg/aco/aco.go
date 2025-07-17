package aco

import (
	"math"
	"math/rand"
)

type Node interface {
	// ID - any unique identifier for this node.
	ID() string
	// Connections - edges connected to this node that are accessible by the ants i.e. available next paths.
	// If there are unidirectional edges of which this node is a destination - they should not be included here.
	Connections() []Edge
	// FitnessValue estimates how far from the final destination this node is.
	// The lower this value is - the smaller the distance i.e. 0 when arrived.
	//
	// route - the route the ant has travelled so far.
	FitnessValue(route []Edge) float64
}

// Edge can be bidirectional, so make sure you don't rely on From() To() meaning strictly "from" -> "to"
// unless you ensure it in your implementation. Sometimes it will mean "from" <-> "to".
type Edge interface {
	// ID - any unique identifier for this edge.
	ID() string
	// From - where the path starts i.e. one of the nodes connected by this edge.
	From() Node
	// To - where the path ends i.e. one of the nodes connected by this edge.
	To() Node
	// Distance - a distance of this path that is visible to the ant.
	Distance() float64
}

type Algorithm interface {
	// FindOptimalRoute looks for the shortest route to the node with the minimum (closest to 0) FitnessValue.
	// Returns the route (slice of paths) and the fitness value of the final node.
	FindOptimalRoute(start Node) ([]Edge, float64)
}

type NodeWrapper interface {
	// Unwrap allows clients to get the inner specific implementation of the Node after the ACO algorithm completes.
	// WARNING: do not use unwrapped node with any ACO lib functions.
	Unwrap() Node
}

type nodeWrapper struct {
	Node
	connectionsWrapped []Edge
}

type edgeWrapper struct {
	Edge
	fromWrapped Node
	toWrapped   Node

	// pheromone is the amount currently present on the edge.
	pheromone float64
	// depositCache stores pheromone deposited within the currently active iteration;
	// at the end of the iteration the pheromone will be updated with the depositCache.
	depositCache float64
}

type algorithm struct {
	conf Configuration
}

func NewAlgorithm(conf Configuration) (Algorithm, error) {
	if err := conf.validate(); err != nil {
		return nil, err
	}
	return &algorithm{conf: conf}, nil
}

func (nw *nodeWrapper) Connections() []Edge {
	if len(nw.connectionsWrapped) > 0 {
		return nw.connectionsWrapped
	}

	conns := nw.Node.Connections()

	wrapped := make([]Edge, len(conns))
	for i, c := range conns {
		wrapped[i] = &edgeWrapper{
			Edge:      c,
			pheromone: 0.0,
		}
	}

	nw.connectionsWrapped = wrapped
	return wrapped
}

func (nw *nodeWrapper) Unwrap() Node {
	return nw.Node
}

func (ew *edgeWrapper) From() Node {
	if ew.fromWrapped == nil {
		ew.fromWrapped = &nodeWrapper{Node: ew.Edge.From()}
	}
	return ew.fromWrapped
}

func (ew *edgeWrapper) To() Node {
	if ew.toWrapped == nil {
		ew.toWrapped = &nodeWrapper{Node: ew.Edge.To()}
	}
	return ew.toWrapped
}

func (a *algorithm) calcWeight(e Edge) float64 {
	ew := e.(*edgeWrapper)
	return math.Pow(ew.pheromone, a.conf.PheromoneImportanceWeight) +
		math.Pow(e.Distance(), a.conf.VisibilityImportanceWeight)
}

func selectRandom(probabilities []float64) int {
	cumulative := make([]float64, len(probabilities))
	cumulative[0] = probabilities[0]
	for i := 1; i < len(probabilities); i++ {
		cumulative[i] = cumulative[i-1] + probabilities[i]
	}

	maxR := cumulative[len(cumulative)-1]
	r := rand.Float64() * maxR

	for i, c := range cumulative {
		if r < c {
			return i
		}
	}

	return len(probabilities) - 1
}

func (a *algorithm) selectConnection(conns []Edge) Edge {
	weights := make([]float64, len(conns))
	weightsSum := 0.0
	for i, c := range conns {
		weights[i] = a.calcWeight(c)
		weightsSum += weights[i]
	}

	probabilities := make([]float64, len(conns))
	for i, w := range weights {
		probabilities[i] = w / weightsSum
	}

	selectedI := selectRandom(probabilities)
	return conns[selectedI]
}

func (a *algorithm) depositPheromone(cost float64, route []Edge) {
	p := a.conf.PheromoneDepositStrength
	if cost > 0 {
		p /= cost
	}

	for i, e := range route {
		revI := len(route) - i + 1 // position from the end.
		ep := p * float64(revI)    // edges closer to start should have greater pheromone value to boost the search.

		ew := e.(*edgeWrapper)
		ew.depositCache += ep
	}
}

func (a *algorithm) updatePheromone(activeEdges []Edge) {
	for _, e := range activeEdges {
		ew := e.(*edgeWrapper)
		p := ew.pheromone * (1.0 - a.conf.EvaporationRate)
		if ew.depositCache > 0.0 {
			p += ew.depositCache
			ew.depositCache = 0.0
		}
		ew.pheromone = p
	}
}

// FindOptimalRoute
// path - a synonym to Edge and "connection", an atomic part of a route that can be taken as one ant step;
// route - a sequence of paths that represents a potential solution.
func (a *algorithm) FindOptimalRoute(start Node) ([]Edge, float64) {
	start = &nodeWrapper{Node: start}
	startFv := start.FitnessValue([]Edge{})

	var bestRoute []Edge
	bestRouteFv := -1.0

	var activeEdges []Edge
	activeEdgesIndex := map[string]bool{}

	for iter := 0; iter < a.conf.NumOfIterations; iter++ {

		for ant := 0; ant < a.conf.NumOfAnts; ant++ {

			antPosition := start

			var route []Edge
			routeFv := startFv

			for depth := 0; depth < a.conf.MaxSearchDepth; depth++ {
				if routeFv == 0.0 {
					// the ant has arrived home from the food source; the route is complete.
					break
				}

				conns := antPosition.Connections()
				if len(conns) == 0 {
					// got into an impasse of the graph - the ant is stuck.
					break
				}

				selectedPath := a.selectConnection(conns)
				route = append(route, selectedPath)

				if selectedPath.To().ID() != antPosition.ID() {
					antPosition = selectedPath.To()
				} else {
					antPosition = selectedPath.From()
				}

				routeFv = antPosition.FitnessValue(route)
			}

			a.depositPheromone(routeFv, route)

			if bestRouteFv == -1.0 || routeFv < bestRouteFv || (routeFv == bestRouteFv && len(route) < len(bestRoute)) {
				bestRoute = route
				bestRouteFv = routeFv
			}

			for _, e := range route {
				eID := e.ID()
				_, found := activeEdgesIndex[eID]
				if !found {
					activeEdgesIndex[eID] = true
					activeEdges = append(activeEdges, e)
				}
			}

		}

		if iter < a.conf.NumOfIterations-1 {
			a.updatePheromone(activeEdges)
		}

	}

	return bestRoute, bestRouteFv
}
