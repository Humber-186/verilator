# Root Cause Analysis: V3TSP.cpp Internal Error with `--threads`/`-j` > 1

## Bug Summary

With `--threads N -j M` (N > 1, M > 1), Verilator crashes:

```
%Error: Internal Error: ../V3TSP.cpp:355: No unmarked edges found in tour
```

The crash is non-deterministic (race-dependent) and was reliably reproduced by the original
reporter on a 192-core machine using `-j 192`.

---

## Reproducing the Bug

**Without the patch** (`s_edgeIdNext` is a global static `uint32_t`):

```
verilator -cc --error-limit 100 \
  --x-assign fast --x-initial unique \
  -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  -DPRINTF_COND=1 -DRANDOMIZE -DRANDOMIZE_MEM_INIT -DRANDOMIZE_REG_INIT \
  --trace-fst --assert \
  --threads 8 --trace-threads 2 \
  -j 192 --prefix Vdut -Mdir /tmp/dut_out \
  _repro/dut.v
# → %Error: Internal Error: ../V3TSP.cpp:355: No unmarked edges found in tour
```

**With the patch** (`m_edgeIdNext` is per-graph):

Same command succeeds reliably.

> **Note:** The crash is a race condition and requires sufficient CPU-level parallelism to
> trigger. On machines with fewer cores (e.g., fewer than 32), the race window is small enough
> that the crash may not manifest within a small number of runs. The original reporter used a
> 192-core machine with `-j 192`, providing many simultaneous `tspSort()` invocations and
> reliably triggering the collision.

---

## Response to Developer Comment

> "If the 'fix' works it's just luck, or I'm misunderstanding, as TSP doesn't itself run
> multithreaded."
> — Wilson Snyder (Verilator maintainer)

### TSP *is* called from the thread pool

The claim that "TSP doesn't run multithreaded" needs qualification.  It is correct that a
**single** `tspSort()` invocation is entirely single-threaded internally — no threads are
spawned inside `tspSort()`.  However, **multiple** `tspSort()` calls from *different modules*
can and do run **concurrently** when `-j > 1`.

The call chain is:

```
V3VariableOrder::orderAll()          // main thread
  V3ThreadScope threadScope;         // acquires the MT scope
  for each module modp:
    threadScope.enqueue([modp, ...] {          // dispatches to worker threads
        VariableOrder::processModule(modp, ...)  // VL_MT_STABLE — runs in worker thread
          → tspSortVars()
            → V3TSP::tspSort(states, &sortedStates)   // VL_MT_SAFE — runs in worker thread
    });
  // V3ThreadScope destructor waits for all jobs
```

Key evidence:

1. `V3ThreadScope::enqueue()` submits jobs to the global `V3ThreadPool`.  With
   `verilateJobs` / `-j` > 1, the pool has worker threads that run jobs concurrently.
2. `VariableOrder::processModule()` is annotated **`VL_MT_STABLE`** — explicitly designed for
   concurrent multi-module processing.
3. `V3TSP::tspSort()` is annotated **`VL_MT_SAFE`** in `V3TSP.h` — the maintainers themselves
   declared it safe to call from multiple threads simultaneously.

The `VL_MT_DISABLED_CODE_UNIT` on `V3TSP.cpp` is a Clang thread-safety analysis hint — it
disables the `-fthread-safety` lock-annotation checker *inside that compilation unit*.  It does
**not** exempt the code from actually being called from worker threads, and it does **not**
mean the code is single-threaded at runtime.

---

## Actual Cause of the Crash

### The race condition

The former `static uint32_t V3TSP::s_edgeIdNext` is a plain (non-atomic) global variable
mutated by every call to `TspGraphTmpl::addEdge()`.  With N worker threads all executing
`tspSort()` concurrently, the non-atomic `++s_edgeIdNext` is a **data race** per the C++
memory model — formally undefined behaviour.

On x86 (TSO memory model), the most common manifestation is: two threads read the *same*
value of the counter before either writes back, so they both obtain the same `edgeId` for
their respective new edges.  Since each graph is local to its own thread, this normally
produces **cross-graph ID collisions**, not within-graph ones.

### How cross-graph collisions trigger the crash

The critical path is `combineGraph()`:

```cpp
void combineGraph(const TspGraphTmpl& g) {
    std::unordered_set<uint32_t> edges_done;   // local dedup set
    for (edge in g.edges()) {
        if (edges_done.insert(getEdgeId(edge)).second)  // seen this logical edge?
            addEdge(...);   // adds the edge to *this, consuming one more edgeId
    }
}
```

Called as `minGraph.combineGraph(matching)`:

- `matching` is built by `perfectMatching()` using `graph.addEdge()` (the *complete* graph's
  edges, not `minGraph`'s).  Each logical edge is stored as two directed edges with the same
  `edgeId`.
- `edges_done` deduplicates the two directed representations of the same logical edge.

With the global race, two distinct logical edges in the *same* `matching` graph can
receive the same `edgeId` if their `addEdge()` calls suffer a **lost-update** race: two
threads simultaneously read the same value of `s_edgeIdNext`, both increment locally, and
both store the same result — a classic read-modify-write (RMW) race.  Under C++ UB semantics,
the compiler may also optimize away repeated loads from a non-atomic global, making the
collision even more likely on the same module's matching graph.

When two logical matching edges share an `edgeId`, `combineGraph` treats the second as a
duplicate and **silently drops it**.  The dropped edge leaves one or more vertices with odd
degree in `minGraph`.  The Euler tour (`findEulerTourRecurse`) requires all vertices to have
even degree.  When it reaches a vertex whose remaining edges have all been "marked" by the
ID collision, it hits:

```cpp
v3fatalSrc("No unmarked edges found in tour");
```

### Why the per-graph fix is correct

Making `m_edgeIdNext` an instance variable of `TspGraphTmpl` (initialised to 0 for every
new graph object) means:

- **No shared mutable state between threads.**  Each `tspSort()` call creates its own
  `Graph graph`, `Graph minGraph`, `Graph matching` — all on the stack/heap of the calling
  thread.  Their `m_edgeIdNext` counters are entirely private.
- **IDs are unique within each graph.**  A sequential, single-threaded increment of a
  per-object field cannot produce duplicates.
- **`combineGraph` deduplication is correct.**  The `edges_done` set can now reliably
  distinguish two directed edges of the *same* logical edge from two *distinct* logical edges.

This is both the minimal correct fix and the right design: graph-local ID uniqueness is all
that is needed, and a global counter was unnecessary to begin with.

---

## Fix

`src/V3TSP.cpp`:

```diff
-namespace V3TSP {
-static uint32_t s_edgeIdNext = 0;
-
-static void selfTestStates();
+namespace V3TSP {
+static void selfTestStates();

 // MEMBERS
 std::unordered_map<T_Key, Vertex*> m_vertices;
+uint32_t m_edgeIdNext = 0;  // Next per-graph edgeId; must be unique within this graph

-const uint32_t edgeId = ++V3TSP::s_edgeIdNext;
+const uint32_t edgeId = ++m_edgeIdNext;
```
