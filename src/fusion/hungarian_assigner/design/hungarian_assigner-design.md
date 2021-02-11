hungarian_assigner {#hungarian-assigner-design}
==================


# Purpose / Use cases

We require a method to associate distinct data together.

This can include multiple observations, but the main use case is to associate
observations with tracks, that is, tracked objects.

The Hungarian algorithm was chosen instead of the Global Nearest Neighbors (GNN)
algorithm because the GNN algorithm assumes a euclidean cost function, whereas
the Hungarian algorithm can take in arbitrary costs.

The primary trade-off here is that the Hungarian algorithm is O(N^3) instead of
O(N log N) for GNN.


# Design

This package is a fairly standard implementation of the O(N^3 ) Hungarian
algorithm (see external links), however we have made some minor changes in
book-keeping to save some work:

- Weights can and should be set in parallel, where a thread has ownership over a
whole row(s)
- Any particular assignment should get its weight set only once (or at least
with decreasing weights for subsequent set operations). This allows us to know
the minimum weight in a row
- All loops are strictly bounded, which adds some computational effort in
exchange for a strict runtime
- Uncovered rows/columns are only found once per outer loop

In addition, our implementation of the Hungarian algorithm can support the
unbalanced assignment problem and missing weights (that is, illegal
assignments).


## Matrices

The following functionality for a matrix class is used:

- Fixed-sized matrices
- Indexing into matrices
- Matrix/array sub-blocks/segments
- Operations on blocks: assignment, addition, subtraction (in place)


## Assumptions / Known limits

- Fixed maximum capacity via template parameter, can choose one of:

    - 16
    - 32
    - 64
    - 96
    - 128
    - 192
    - 256

- It is assumed that the user knows the number of workers and tasks (rows and
  columns) BEFORE filling the matrix via `set_weight()`
- It is assumed that the user knows the number of tasks that will be unassigned
- It is assumed that `set_weight()` for a given index pair occurs only once
- **The user must set rows and columns such that the number of columns is >=
number of rows**
- If a row has no possible assignments and/or `assign()` fails and returns
false, it is assumed that the user will then need to check for an
`UNASSSIGNED` assignment


## Inputs / Outputs / API

Basic usage:

```{cpp}
// set size
assigner.set_size(16U, 16U);
// set weights
assigner.set_weight(0.0F, 0U, 0U);
// ... for impossible assignments, don't set weight
const bool8_t ret = assigner.assign();  // do assignment
// get results
const uint64_t idx = assigner.get_assignment(0U);
const uint64_t jdx = assigner.get_unassigned(0U);
// reset before doing stuff again
assigner.reset();
assigner.set_size(32U, 32U);
// continue...
```

Inputs:

- Size of problem
- Assignment weights (+implicit impossible assignment)

Outputs:

- Assignments (mapping from row to assigned column)
- Unassigned tasks (which columns are unassigned columns)


# Error detection and handling

Impossible assignments are detected by hitting loop bounds. In some cases, we
can also check before doing computation if assignment is impossible (e.g. if a
column is completely empty).

In the above case, the full algorithm will run, but `assign()` will return
false. This is a signal to the user that they need to check the result of
`get_assignment()` for the value `UNASSIGNED`

For indexing (e.g. `set_size()`, `set_weight()`), exceptions can be thrown due
to indexing errors.

For some inner loops, certain conditions should be theoretically guaranteed
(e.g. maximum size of augmented path, maximum number of iterations for loops,
  etc.).

If these conditions are not met, an exception will be thrown.


# Security considerations

TBD by a security specialist.


# References / External links

- [Baidu's Apollo Reference implementation](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/common/graph/hungarian_optimizer.h)
- [Prose description of algorithm](https://stackoverflow.com/questions/23278375/hungarian-algorithm)
- [Worked example for unit test 1](http://naagustutorial.blogspot.com/2013/12/hungarian-method-unbalanced-assignment.html)
- [Worked example for unit test 2](http://file.scirp.org/pdf/AJOR_2016063017275082.pdf)

# Future extensions / Unimplemented parts

N/A


# Related issues

- #1152: Initial implementation
- #1266: Refactor to remove eigen dependency
- #1337: Support for partial assignments
- #1394: Remove redundant ament calls
- #1559: Update the design doc format to match the documentation style guide
