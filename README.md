# DESIGN.md – Sensor Filtering (MA + Kalman)

## 1. Overview

This document describes the design decisions, algorithmic proofs, and
resource analysis for the two sensor filters implemented for the ICARUS
1U CubeSat flight software.

---

## 2. Moving Average Filter

### 2.1 Algorithm

A **circular buffer** of fixed size `WINDOW_SIZE = 8` stores the most
recent samples. A single `float sum` tracks the running total so that
the average can be computed without iterating over the buffer.

**Per-sample steps:**

1. Subtract `buffer[index]` (the oldest slot) from `sum`.
2. Write the new sample into `buffer[index]`.
3. Add the new sample to `sum`.
4. Advance `index = (index + 1) % WINDOW_SIZE` — wraps with one modulo, no branch.
5. Return `sum / count`, where `count` is capped at `WINDOW_SIZE`.

### 2.2 O(1) Proof

Every step above is a single arithmetic or assignment operation
independent of `WINDOW_SIZE` or the total number of samples received:

| Step | Operations |
|------|------------ |
| Subtract old value | 1 subtraction |
| Write new sample | 1 assignment |
| Add new sample | 1 addition |
| Advance index | 1 addition + 1 modulo |
| Compute average | 1 division |

**Total: O(1) — constant regardless of window size or input length.**

There are no loops, no recursion, and no data-dependent branching inside
`moving_average_update`.

### 2.3 Fill Phase & Edge Cases

| Scenario | Behaviour |
|----------|-----------|
| First sample | `count = 1`; average equals that sample |
| Fewer than `WINDOW_SIZE` samples received | Divides by `count`, not `WINDOW_SIZE`, giving an unbiased mean |
| Buffer full | `count` is capped at `WINDOW_SIZE`; normal sliding-window behaviour |
| NULL pointer passed | Both `init` and `update` return immediately; `update` returns `0.0f` |

---

## 3. Kalman Filter (Single-State)

### 3.1 Algorithm

The scalar Kalman filter fuses a **process model** (predict step) with a
**noisy measurement** (update step) using a gain computed from
uncertainty covariances.

**Predict:**
```
x_pred = x          (constant model — no control input)
P_pred = P + Q      (uncertainty grows with process noise Q)
```

**Update:**
```
S = P_pred + R      (innovation covariance — total uncertainty)
K = P_pred / S      (Kalman gain: 0→trust model, 1→trust sensor)
x = x_pred + K * (z - x_pred)
P = (1 - K) * P_pred
```

Where `z` is the incoming measurement.

### 3.2 O(1) Proof

| Step | Operations |
|------|------------ |
| Predict P | 1 addition |
| Compute S | 1 addition |
| Compute K | 1 division |
| Update x | 1 subtraction + 1 multiplication + 1 addition |
| Update P | 1 subtraction + 1 multiplication |

**Total: O(1) — five scalar arithmetic operations, no loops.**

### 3.3 Parameter Guide

| Parameter | Effect |
|-----------|--------|
| High `Q` | Filter tracks rapid changes; more responsive but noisier |
| Low `Q` | Filter is sluggish but smooth |
| High `R` | Measurements are distrusted; model dominates |
| Low `R` | Measurements are trusted; converges quickly |

### 3.4 Edge Cases

| Scenario | Behaviour |
|----------|-----------|
| `P = 0, R = 0` → `S = 0` | Gain undefined; update is skipped, current estimate returned |
| Negative `P`, `Q`, or `R` supplied to `init` | Clamped to `0.0f` — prevents negative covariances that would break filter stability |
| NULL pointer | Both `init` and `update` return immediately; `update` returns `0.0f` |

---

## 4. RAM Usage Report

All memory is **stack-allocated** (no `malloc`/`free`).

### Filter Structs (in `tests/test_filters.c` and any flight module)

| Variable | Size (bytes) |
|----------|-------------|
| `MovingAverageFilter.buffer` | `8 × 4 = 32` |
| `MovingAverageFilter.sum` | `4` |
| `MovingAverageFilter.index` | `1` |
| `MovingAverageFilter.count` | `1` |
| *(padding)* | `2` |
| **MovingAverageFilter total** | **40** |
| `KalmanFilter` (4 × float) | `16` |
| **Both filters combined** | **56 bytes** |

### Test-Suite Stack (worst case, one frame at a time)

| Locals in test functions | ≤ 32 bytes |
|--------------------------|------------|

### Total Estimated RAM

| Section | Bytes |
|---------|-------|
| Filter instances | 56 |
| Local variables / stack | ≤ 64 |
| **Grand total** | **≤ 120 bytes** |

**Target: < 1 024 bytes ✓ (estimated usage ≤ 120 bytes, ~12% of budget)**

---

## 5. Complexity Summary

| Function | Time | Space |
|----------|------|-------|
| `moving_average_init` | O(WINDOW\_SIZE) | O(1) extra |
| `moving_average_update` | **O(1)** | O(1) extra |
| `kalman_init` | O(1) | O(1) extra |
| `kalman_update` | **O(1)** | O(1) extra |

> `moving_average_init` is O(WINDOW\_SIZE) only because it zero-fills the
> fixed-size buffer; this is a one-time setup cost, not per-sample.

---

## 6. How to Build and Run

All development must be done inside the provided Docker container (Ubuntu 22.04).
Do **not** compile directly on your local machine.

### Prerequisites
- [Docker Desktop](https://www.docker.com/products/docker-desktop) installed and running.

### Step 1 — Navigate to the project folder
Open a terminal and `cd` into the root of the project (the folder containing the `Dockerfile`):
```bash
cd /path/to/icarus_assignment
```

### Step 2 — Build the Docker image (one-time setup)
```bash
docker build -t icarus-c-dev .
```
This downloads Ubuntu 22.04 and installs all required tools (`gcc`, `gdb`, `valgrind`, etc.).
Only needs to be run once.

### Step 3 — Start the container
```bash
docker run -it --rm -v $(pwd):/workspace icarus-c-dev
```
Your prompt will change to `root@<id>:/workspace#` — you are now inside Ubuntu.
Your local project files are mounted at `/workspace` and stay in sync with your host machine.

### Step 4 — Compile
```bash
make
```

### Step 5 — Run the test suite
```bash
make run
```
Expected output: `Results: 33 / 33 tests passed`

### Other useful commands

| Command | Description |
|---------|-------------|
| `make clean` | Remove the compiled binary |
| `make debug` | Open the binary in `gdb` |
| `valgrind ./main` | Check for memory errors |
| `exit` | Leave the container and return to your local terminal |

> **Note:** Any edits made to the files on your local machine are instantly
> visible inside the container — no need to restart it.

---

## 7. Design Constraints Checklist

| Constraint | Status |
|------------|--------|
| Language: C only | ✅ |
| No dynamic allocation (`malloc`/`free`) | ✅ |
| O(1) per sample | ✅ |
| RAM < 1 KB | ✅ (≤ 120 bytes) |
| Struct definitions match specification | ✅ |
| Function signatures match specification | ✅ |
