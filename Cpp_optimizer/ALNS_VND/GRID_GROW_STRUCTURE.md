# Grid-Grow Algorithm Structure

This document outlines the structure of the **Grid-Grow ALNS** algorithm implemented in `grid_grow_alns.h`. This algorithm is designed for cluster-trip optimization, treating routes as spatial clusters on a grid.

## Core Hierarchy

The algorithm operates on a few key data structures roughly corresponding to the physical entities:

1.  **State (`gg_alns::State`)**: Represents the entire solution.
    *   **Grid Index (`GridIndex`)**: Maps spatial cells to customers for fast neighborhood lookups.
    *   **Clusters (`Cluster`)**: Represents a trip/route. Maintains:
        *   `members`: List of customers in the cluster.
        *   `boundary`: Customers on the geometric edge of the cluster (used for expansion/peeling).
        *   `center_ui` & `trip_km`: Cached representative center and trip cost.
    *   **Transactional Methods**: `try_insert`, `try_move`, `try_swap`, `remove_from_cluster`, `add_to_cluster`. All modifications are tracked to allow rollback if constraints (hard) or penalties (soft) are violated.
2.  **Hooks (`Hooks`)**: Callbacks for external constraints (e.g., Vehicle Capacity, Max Distance, Depot Load). These allow the core logic to remain generic while enforcing specific business rules.

## Algorithm Flow (`GridGrowALNS::run`)

The main loop runs for a fixed number of iterations (`iters`) using a Simulated Annealing (SA) framework.

### 1. Destroy Step (Ruins)
The algorithm selects a destroy operator based on adaptive weights (Roulette Wheel Selection):
*   **Boundary Peel (`DestroyOps::boundary_peel`)**: Removes a fraction of customers acting as the "skin" or boundary of a cluster. Useful for reshaping.
*   **Region Removal (`DestroyOps::connected_region_cells`)**: Removes a spatially connected chunk of cells from a cluster. Useful for splitting or major reshaping.
*   **Center Shift (`DestroyOps::center_shift`)**: Forces a new center selection for a cluster to explore different geometric configurations.

### 2. Repair Step (Recreate)
*   **Regrow (`GrowOps::regrow_cluster`)**: The primary constructive heuristic.
    *   It picks the best candidates to add back to a cluster.
    *   **Candidate Selection (`pick_best_candidate`)**: Scores unassigned customers based on:
        *   Distance to cluster centroid.
        *   Distance to cluster center (depot trip point).
        *   Spatial connectivity (proximity to existing boundary cells).
    *   **Frontier Expansion**: It naturally grows the cluster outwards from its boundary, mimicking a flood-fill or crystallization process.

### 3. Local Search (VND)
A Variable Neighborhood Descent (VND) phase tightens the boundaries between clusters:
*   **Boundary Relocate**: Moves a boundary customer from Cluster A to Cluster B if it improves cost.
*   **Boundary Swap**: Swaps boundary customers between touching clusters A and B.
*   *Note: This only considers spatially adjacent clusters to keep complexity low.*

## Key Files for Analysis

*   **`grid_grow_alns.h`**: **(CRITICAL)** Contains the entire logic described above.
    *   `struct State`: Data model and atomic moves.
    *   `struct GrowOps`: Logic for scoring and adding customers (The "Grow" part).
    *   `struct DestroyOps`: Logic for removing customers.
    *   `class GridGrowALNS`: The main loop handling ALNS weights and SA acceptance.
*   **`initialization.h`**: Contains `Initializer::create_initial_solution`.
    *   Concepts here (Seed selection, Ring growth) are very similar to `grid_grow_alns.h` but used for creating the *first* valid solution. It handles the initial "filling" of vehicles.
*   **`alns_vnd.h`**: (If used) Seems to be a larger/alternative implementation binding everything together, likely using `grid_grow_alns.h` concepts or serving as the production controller.

## Logic Summary
> "Grow from seeds, peel from edges, swap at boundaries."

The algorithm avoids complex global moves (like arbitrary relocations) and instead focuses on **geometric operations**: adding to the edge (grow) and removing from the edge (peel). This makes it highly effective for spatial clustering problems where "compactness" is key.
