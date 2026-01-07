# Grid-Grow ALNS Algorithm - Technical Documentation

## 1. Tổng Quan (Overview)

Thuật toán **Grid-Grow ALNS** (Adaptive Large Neighborhood Search) là một meta-heuristic được thiết kế đặc biệt cho bài toán tối ưu cluster-trip (chuyến đi theo cụm) trong môi trường không gian 2D. Thuật toán kết hợp ba thành phần chính:

1. **Spatial Grid Indexing**: Lập chỉ mục không gian dựa trên lưới rời rạc
2. **ALNS Framework**: Khung metaheuristic với destroy/repair operators thích nghi
3. **VND Local Search**: Tìm kiếm cục bộ Variable Neighborhood Descent

### 1.1 Định Nghĩa Bài Toán

Cho:
- Tập khách hàng $C = \{c_1, c_2, ..., c_n\}$ với tọa độ $(x_i, y_i)$
- Tập depot $D = \{d_1, d_2, ..., d_m\}$
- Tập phương tiện $V = \{v_1, v_2, ..., v_k\}$ với capacity $Q_v$

Mục tiêu: Phân chia khách hàng thành các cluster (cụm), mỗi cluster được phục vụ bởi một phương tiện trong một chuyến đi từ depot → cluster center → depot, sao cho:
- Minimize: Tổng chi phí di chuyển và số khách hàng không được phục vụ
- Subject to:
  - Ràng buộc capacity xe: $\sum_{c_i \in \text{cluster}} w_i \leq Q_v$
  - Ràng buộc spread (độ phân tán): $\max_{c_i, c_j \in \text{cluster}} d(c_i, c_j) \leq R_{max}$
  - Ràng buộc khoảng cách tổng: $\sum_{\text{trips}} 2 \times d(\text{depot}, \text{center}) \leq D_{max}$

---

## 2. Cấu Trúc Dữ Liệu (Data Structures)

### 2.1 Grid Index Structure

```cpp
struct GridIndex {
    double cell_size_km = 0.02;  // Mỗi cell = 20m x 20m
    vector<CellKey> cust_cell;   // Ánh xạ customer → cell key
    unordered_map<CellKey, vector<int>> cell2cust; // cell → danh sách customers
};
```

**Công thức chuyển đổi tọa độ sang cell:**

$$\text{CellKey}(x, y) = \left( \left\lfloor \frac{y}{\text{cell\_size}} \right\rfloor, \left\lfloor \frac{x}{\text{cell\_size}} \right\rfloor \right)$$

**Ý nghĩa:** Chia không gian liên tục thành lưới rời rạc giúp tra cứu láng giềng $O(1)$ thay vì $O(n)$.

### 2.2 Cluster Structure

```cpp
struct Cluster {
    int vehicle_id, depot_id;          // Phương tiện và depot
    vector<int> members;                // Danh sách khách hàng trong cluster
    
    // Cached statistics (được cập nhật incremental)
    double sumx, sumy;                  // Tổng tọa độ (cho centroid)
    double load_w, load_vol;            // Tải trọng tổng
    
    int center_ui;                      // Khách hàng đại diện (center)
    double trip_km;                     // = 2 × d(depot, center)
    
    vector<int> boundary;               // Danh sách customers ở biên
    bool boundary_dirty;                // Flag cần recompute boundary
};
```

**Centroid Calculation:**

$$\text{centroid} = \left( \frac{\sum_{i \in \text{members}} x_i}{|\text{members}|}, \frac{\sum_{i \in \text{members}} y_i}{|\text{members}|} \right)$$

**Center Selection:** Khách hàng gần centroid nhất thỏa mãn:
1. Tất cả customers khác trong cluster cách center ≤ $R_{max}$ (spread constraint)
2. Edge từ depot → center được phép (edge_allowed constraint)

### 2.3 State Structure

```cpp
struct State {
    const vector<Customer>* cust;
    const vector<Depot>* depots;
    const vector<Vehicle>* veh;
    
    GridIndex grid;
    Hooks hooks;                        // Constraint callbacks
    
    vector<Cluster> clusters;           // Danh sách clusters hiện tại
    vector<int> cust_cluster;           // customers[i] → cluster_id (-1 nếu unassigned)
    vector<int> unassigned;             // Danh sách customers chưa phục vụ
    
    // Totals (cached incrementally)
    vector<double> depot_total_load;    // Load tại mỗi depot
    vector<double> vehicle_total_km;    // Km đã dùng của mỗi xe
    
    double total_cost;                  // Objective value
    int unassigned_cnt;                 // Số khách hàng unassigned
};
```

**Objective Function:**

$$\text{total\_cost} = \sum_{\text{clusters}} \text{trip\_km} + \text{unassigned\_cnt} \times 10^8 + \sum \text{penalties}$$

Trong đó:
- $\text{trip\_km}$: Chi phí di chuyển
- $\text{unassigned\_cnt} \times 10^8$: Penalty rất lớn cho khách hàng không phục vụ (ưu tiên phục vụ tối đa)
- $\text{penalties}$: Penalty cho capacity overflow (soft constraint)

---

## 3. Thuật Toán Chính (Main Algorithm)

### 3.1 ALNS Framework Pseudocode

```
Algorithm: GridGrowALNS(initial_state, max_iterations)
Input: 
    - initial_state: Solution ban đầu
    - max_iterations: Số vòng lặp
Output: Best solution found

1. Initialize:
   current ← initial_state
   best ← current
   T ← T0                          // Initial temperature (SA)
   weights ← [1, 1, 1]             // Adaptive weights cho destroy ops
   
2. For iter = 1 to max_iterations:
   
   a) backup ← current             // Backup để rollback nếu cần
   
   b) DESTROY PHASE:
      - Select destroy_op using roulette wheel (weights)
      - Apply destroy_op to remove customers from clusters
      
   c) REPAIR PHASE:
      - Regrow affected cluster(s) using GrowOps
      - Regrow random clusters (diversification)
      
   d) LOCAL SEARCH (VND):
      - Apply boundary_relocate and boundary_swap
      
   e) ACCEPTANCE CRITERION (Simulated Annealing):
      Δ ← current.cost - backup.cost
      If Δ ≤ 0 OR rand() < exp(-Δ/T):
          Accept current
          Update weights (reward successful operator)
          If current.cost < best.cost:
              best ← current
      Else:
          current ← backup  // Rollback
   
   f) UPDATE PARAMETERS:
      T ← T × α                    // Cooling schedule (α = 0.995)
      Update adaptive weights every 50 iterations
      
3. Return best
```

### 3.2 Các Tham Số Quan Trọng

| Tham số | Giá trị mặc định | Ý nghĩa |
|---------|------------------|---------|
| `cell_size_km` | 0.02 km (20m) | Kích thước mỗi cell trong grid |
| `T0` | 0.05 | Nhiệt độ ban đầu (SA) |
| `alpha` | 0.995 | Hệ số làm lạnh (cooling rate) |
| `grow_budget` | 64 | Số lần thử tối đa khi grow một cluster |
| `vnd_iters` | 60 | Số iteration VND |
| `peel_frac` | 0.25 | Tỉ lệ boundary customers bị remove |
| `region_cells` | 8 | Số cells trong connected region removal |

---

## 4. Destroy Operators (Phá Hủy)

### 4.1 Boundary Peel

**Ý tưởng:** Gọt bỏ các khách hàng ở biên của cluster để tạo cơ hội reshape.

**Thuật toán:**
```
BoundaryPeel(cluster, peel_fraction):
1. Compute boundary customers:
   boundary = customers có ít nhất 1 neighbor cell thuộc cluster khác
   
2. Select k = peel_fraction × |boundary| customers randomly
3. For each selected customer c:
   - Remove c from cluster
   - If cluster still feasible: commit removal
   - Else: rollback (re-insert c)
```

**Độ phức tạp:** $O(k \times n_c)$ với $n_c$ = số customers trong cluster

### 4.2 Connected Region Removal

**Ý tưởng:** Xóa một vùng liên thông (theo cells) để tạo "lỗ hổng" lớn trong cluster.

**Thuật toán BFS:**
```
ConnectedRegionCells(cluster, target_cells):
1. Build cell_members map: cell → customers trong cluster
2. seed ← random boundary cell
3. BFS expand từ seed với 8-neighbor connectivity:
   - Queue ← {seed}
   - Visited ← {seed}
   - While |Visited| < target_cells AND Queue not empty:
       current ← Queue.pop()
       For each neighbor ∈ 8_neighbors(current):
           If neighbor in cluster AND neighbor not visited:
               Visited.add(neighbor)
               Queue.add(neighbor)
4. Remove all customers in Visited cells (với rollback)
```

**Độ phức tạp:** $O(\text{target\_cells} \times 8 + k \times n_c)$

### 4.3 Center Shift

**Ý tưởng:** Thay đổi center (điểm đại diện) của cluster để thay đổi trip_km và hình dạng cluster.

**Thuật toán:**
```
CenterShift(cluster):
1. Pick random customer ≠ current center
2. Check if new_center satisfies:
   - All customers in cluster are within R_max from new_center
   - Edge(depot, new_center) is allowed
3. If feasible:
   - Update cluster.center ← new_center
   - Update cluster.trip_km ← 2 × d(depot, new_center)
```

**Độ phức tạp:** $O(n_c)$ để check spread constraint

---

## 5. Repair Operator (Tái Tạo)

### 5.1 Grow Operations - Candidate Scoring

**Mục tiêu:** Chọn khách hàng tốt nhất để thêm vào cluster.

**Scoring Function:**

$$\text{score}(c_i) = w_{\text{centroid}} \cdot d(c_i, \text{centroid}) + w_{\text{center}} \cdot d(c_i, \text{center}) + w_{\text{cell}} \cdot \text{cell\_bonus}(c_i)$$

Trong đó:
- $d(c_i, \text{centroid})$: Khoảng cách tới tâm cluster (khuyến khích compact)
- $d(c_i, \text{center})$: Khoảng cách tới center (representative point)
- $\text{cell\_bonus}(c_i) = -1$ nếu $c_i$ nằm trong cell kề với boundary customer, 0 otherwise

**Trọng số mặc định:**
- $w_{\text{centroid}} = 1.0$
- $w_{\text{center}} = 0.7$
- $w_{\text{cell}} = 0.3$

### 5.2 Frontier Expansion (Mở Rộng Biên)

**Thuật toán Regrow:**

```
RegrowCluster(cluster, budget):
1. For iteration = 1 to budget:
   
   a) Collect candidates:
      If cluster is empty:
          - Search 8-neighbors around depot cell
          - Fallback: search ALL unassigned customers globally
      Else:
          - Search 8-neighbors around all boundary cells
   
   b) Score all candidates using scoring function
   
   c) best ← candidate with lowest score satisfying:
      - Hard constraints (capacity, vehicle type compatibility)
      - Soft constraints (spread, max_dist) with tolerance
   
   d) If best exists:
      - try_insert(best, cluster)
      - If insertion successful: continue
      - Else: break
   
   e) Else: break (no more feasible candidates)
```

**Đặc điểm:**
- **Greedy:** Chọn best candidate mỗi iteration
- **Frontier-based:** Chỉ xem xét customers gần biên (không scan toàn bộ)
- **Transactional:** Mỗi insertion được validate, rollback nếu vi phạm

**Độ phức tạp:** $O(\text{budget} \times |\text{frontier}| \times n_c)$

### 5.3 Transactional Insert

**Thuật toán try_insert:**

```
try_insert(customer_ui, cluster_id):
1. Check hard constraints:
   - customer not already assigned
   - cluster is active
   - vehicle type compatible with customer class
   - capacity not exceeded (hard limit)

2. Tentative addition:
   - new_load ← cluster.load + customer.weight
   - new_center ← recompute_center({cluster.members + customer})
   - new_trip_km ← 2 × d(depot, new_center)

3. Check soft constraints (với rollback):
   - Spread: max_distance_to_center ≤ R_max
   - Max dist: vehicle_total_km + new_trip_km ≤ D_max
   - Depot capacity (soft penalty)

4. If all checks pass:
   - Add customer to cluster.members
   - Update cluster stats (sumx, sumy, load)
   - Update depot_total_load, vehicle_total_km
   - Finalize cluster (recompute center)
   - Update objective incrementally
   - Return TRUE
   
5. Else:
   - Rollback all changes
   - Return FALSE
```

**Tính chất quan trọng:** Đảm bảo State luôn consistent (không có "half-done" operations).

---

## 6. VND Local Search

### 6.1 Boundary Relocate

**Ý tưởng:** Di chuyển một customer từ biên của cluster A sang cluster B lân cận.

**Thuật toán:**
```
BoundaryRelocate():
1. Pick two spatially adjacent clusters A, B
2. Select random customer u from boundary(A)
3. Attempt: try_move(u, A → B)
4. If move successful AND cost improved: accept
5. Else: rollback
```

**Điều kiện "spatially adjacent":** Tồn tại $u \in A, v \in B$ sao cho các cell của chúng kề nhau (8-neighbor).

### 6.2 Boundary Swap

**Ý tưởng:** Hoán đổi hai customers ở biên của hai clusters lân cận.

**Thuật toán:**
```
BoundarySwap():
1. Pick two spatially adjacent clusters A, B
2. Select random u ∈ boundary(A), v ∈ boundary(B)
3. Attempt: try_swap(u ↔ v)
4. Check feasibility:
   - Remove u from A, v from B
   - Insert v into A, u into B
   - All operations must satisfy constraints
5. If swap successful AND cost improved: accept
6. Else: rollback
```

**Độ phức tạp mỗi iteration:** $O(n_c)$ với $n_c$ = average cluster size

---

## 7. Simulated Annealing Acceptance

**Acceptance Probability:**

$$P(\text{accept}) = \begin{cases}
1 & \text{if } \Delta \leq 0 \\
e^{-\Delta / T} & \text{if } \Delta > 0
\end{cases}$$

Trong đó:
- $\Delta = \text{cost}_{\text{new}} - \text{cost}_{\text{current}}$
- $T$: Nhiệt độ hiện tại

**Cooling Schedule:**

$$T_{t+1} = \alpha \cdot T_t$$

Với $\alpha = 0.995$ (slow cooling để exploration tốt hơn).

**Tác dụng:**
- Đầu iteration: $T$ cao → chấp nhận nhiều worse solutions → exploration
- Cuối iteration: $T$ thấp → chỉ chấp nhận improvements → exploitation

---

## 8. Adaptive Weights (Học Tự Động)

**Mục tiêu:** Động điều chỉnh xác suất chọn các destroy operators dựa trên hiệu quả.

**Scoring System:**
```
For each iteration:
1. Track which destroy_op was used
2. Reward based on result:
   - If new best found: score += 5.0
   - If improvement: score += 2.0
   - If accepted (no improvement): score += 0.5

Every 50 iterations:
3. Update weights:
   w_new[op] = (1-ρ) × w_old[op] + ρ × (score[op] / usage[op])
   (ρ = 0.2: learning rate)
```

**Ý nghĩa:** Operators hiệu quả cao sẽ được chọn nhiều hơn → adaptive search.

---

## 9. Độ Phức Tạp Tổng Thể

**Mỗi ALNS iteration:**

| Component | Complexity |
|-----------|-----------|
| Destroy | $O(k)$ với $k$ = số customers removed |
| Repair (Regrow) | $O(\text{budget} \times \|\text{frontier}\| \times n_c)$ |
| VND | $O(\text{vnd\_iters} \times n_c)$ |
| Overhead | $O(1)$ (incremental updates) |

**Overall per iteration:** $O(\text{budget} \times B \times n_c)$ với $B$ = average boundary size

**Toàn bộ thuật toán:** $O(\text{max\_iterations} \times \text{budget} \times B \times n_c)$

**Trong thực tế:**
- Grid indexing làm giảm $B$ đáng kể (chỉ xét local neighbors)
- Incremental updates cho phép validate nhanh
- Early termination trong regrow giảm số operations

---

## 10. Các File Cài Đặt Quan Trọng

### 10.1 `grid_grow_alns.h` ⭐ (CORE FILE)

**Chứa:**
- `struct GridIndex`: Spatial indexing 
- `struct Cluster`: Cluster data structure
- `struct State`: Complete solution representation
- `struct Hooks`: Constraint callbacks
- `struct GrowOps`: Repair logic (frontier expansion, scoring)
- `struct DestroyOps`: Destroy logic (peel, region, shift)
- `struct VND`: Local search (relocate, swap)
- `class GridGrowALNS`: Main ALNS loop

**Số dòng code:** ~1068 lines

### 10.2 `initialization.h`

**Chứa:**
- `Initializer::create_initial_solution()`: Constructive heuristic ban đầu
- Sử dụng tư tưởng tương tự: seed selection + ring expansion
- Khác biệt: Tạo solution từ scratch, không phải optimize existing solution

**Liên hệ với Grid-Grow:**
- Cả hai đều dùng **greedy seed selection** (heavy-first, far-first)
- Cả hai đều dùng **spatial expansion** (ring-based growth)
- Init tạo baseline, Grid-Grow improve baseline

### 10.3 `alns_vnd.h`

**Chứa:**
- Alternative/extended ALNS implementation
- Có state structure tương tự nhưng phức tạp hơn
- Thường dùng cho production với nhiều constraints hơn

**Gợi ý:** Nếu viết báo cáo, tập trung vào `grid_grow_alns.h` vì nó gọn gàng và rõ ràng hơn.

---

## 11. Điểm Mạnh Của Thuật Toán

1. **Spatial Efficiency:**
   - Grid indexing giảm complexity từ $O(n^2)$ xuống $O(B)$ với $B \ll n$
   - Chỉ xem xét local neighbors → fast

2. **Geometric Intuition:**
   - "Grow from edges, peel from edges" → natural cho spatial clustering
   - Tránh arbitrary relocations → stability

3. **Transactional Safety:**
   - Mọi operation đều có rollback → không bao giờ invalid state
   - Dễ debug, dễ maintain

4. **Adaptive Learning:**
   - Weights tự điều chỉnh → không cần hand-tune operators
   - SA framework → balance exploration/exploitation

5. **Soft Constraints Handling:**
   - Penalty-based → có thể temporarily violate để escape local optima
   - Hooks system → flexible constraint injection

---

## 12. Hướng Phát Triển & Cải Tiến

1. **Parallel ALNS:** Chạy nhiều destroy-repair chains song song
2. **Machine Learning:** Học patterns từ good solutions để guide search
3. **Hybrid with Exact Methods:** Combine với IP solver cho subproblems
4. **Dynamic Grid:** Thay đổi cell_size theo giai đoạn search
5. **Multi-objective:** Xử lý tradeoff giữa cost, service, environment impact

---

## Tài Liệu Tham Khảo

- **ALNS Framework:** Pisinger & Ropke (2010) - "Large Neighborhood Search"
- **Spatial Clustering:** DBSCAN, Grid-based clustering
- **VRP with Clusters:** Cluster-first, route-second heuristics
- **Simulated Annealing:** Kirkpatrick et al. (1983)

---

**Document Version:** 2.0  
**Last Updated:** 2025-12-26  
**Author:** Grid-Grow Development Team
