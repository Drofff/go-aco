package main

import (
	"log"

	"github.com/Drofff/go-aco/internal/example"
)

func main() {
	log.Println("Solving TSP example.")
	res, err := example.SolveTSP()
	if err != nil {
		log.Fatal("failed to solve TSP:", err)
	}

	log.Println(res)
}
