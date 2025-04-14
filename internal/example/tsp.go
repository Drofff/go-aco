package example

import (
	"fmt"
	"strings"

	"github.com/Drofff/go-aco/pkg/aco"
)

type tspNode struct {
	id    string
	conns []aco.Edge

	citiesCount int
}

func (n *tspNode) ID() string {
	return n.id
}

func (n *tspNode) Connections() []aco.Edge {
	return n.conns
}

func (n *tspNode) FitnessValue(route []aco.Edge) float64 {
	cityVisitIndex := map[string]bool{}
	uniqueCitiesVisited := 0
	for _, e := range route {
		_, visited := cityVisitIndex[e.From().ID()]
		if !visited {
			cityVisitIndex[e.From().ID()] = true
			uniqueCitiesVisited++
		}

		_, visited = cityVisitIndex[e.To().ID()]
		if !visited {
			cityVisitIndex[e.To().ID()] = true
			uniqueCitiesVisited++
		}
	}

	return float64(n.citiesCount - uniqueCitiesVisited)
}

type tspEdge struct {
	id   string
	from *tspNode
	to   *tspNode
	dist float64
}

func (e *tspEdge) ID() string {
	return e.id
}

func (e *tspEdge) From() aco.Node {
	return e.from
}

func (e *tspEdge) To() aco.Node {
	return e.to
}

func (e *tspEdge) Distance() float64 {
	return e.dist
}

// SolveTSP is an example of solving the Travelling Salesman Problem for the following graph:
//
// [1] - (10) - [2]
//
//	|       /   |
//
// (15)   (20) (25)
//
//	|   /       |
//
// [3] - (25) - [4]
//
// Cities: [1], [2], [3], [4]
// Roads:
// - [1] to [2] with cost 10
// - [1] to [3] with cost 15
// - [2] to [3] with cost 20
// - [2] to [4] with cost 25
// - [3] to [4] with cost 25
func SolveTSP() (string, error) {
	citiesCount := 4
	node1 := &tspNode{id: "1", citiesCount: citiesCount}
	node2 := &tspNode{id: "2", citiesCount: citiesCount}
	node3 := &tspNode{id: "3", citiesCount: citiesCount}
	node4 := &tspNode{id: "4", citiesCount: citiesCount}

	edge12 := &tspEdge{
		id:   "1-2",
		from: node1,
		to:   node2,
		dist: 10.0,
	}
	edge13 := &tspEdge{
		id:   "1-3",
		from: node1,
		to:   node3,
		dist: 15.0,
	}
	edge23 := &tspEdge{
		id:   "2-3",
		from: node2,
		to:   node3,
		dist: 20.0,
	}
	edge24 := &tspEdge{
		id:   "2-4",
		from: node2,
		to:   node4,
		dist: 25.0,
	}
	edge34 := &tspEdge{
		id:   "3-4",
		from: node3,
		to:   node4,
		dist: 25.0,
	}

	node1.conns = []aco.Edge{edge12, edge13}
	node2.conns = []aco.Edge{edge23, edge24}
	node3.conns = []aco.Edge{edge34}
	node4.conns = []aco.Edge{}

	conf := aco.Configuration{
		NumOfAnts:                  30,
		NumOfIterations:            20,
		PheromoneImportanceWeight:  0.7,
		VisibilityImportanceWeight: 1.0,
		EvaporationRate:            0.5,
		PheromoneDepositStrength:   5,
		MaxSearchDepth:             20,
	}

	a, err := aco.NewAlgorithm(conf)
	if err != nil {
		return "", fmt.Errorf("new aco algorithm: %w", err)
	}

	route, _ := a.FindOptimalRoute(node1)

	var routeNodes []string
	totalDist := 0.0
	for _, e := range route {
		routeNodes = append(routeNodes, "["+e.ID()+"]")
		totalDist += e.Distance()
	}

	return fmt.Sprintf("route: %v; distance: %v.", strings.Join(routeNodes, ","), totalDist), nil
}
