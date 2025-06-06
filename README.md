# Wireless Network Simulation for OFDMA and Multi-Channel Scheduling

This repository contains a C++ implementation of a wireless network simulation for Orthogonal Frequency Division Multiple Access (OFDMA) and multi-channel scheduling, based on IEEE 802.11be Wi-Fi standards. The simulation models an Access Point (AP) managing multiple Stations (STAs) with varying traffic priorities, packet deadlines, and channel conditions, supporting single-link and multi-link operations (MLO). It includes advanced resource allocation using the Hungarian algorithm for optimal STA-MRU assignments.

## Overview

The codebase simulates a wireless network where an AP schedules packet transmissions for STAs using OFDMA over one or two channels (320 MHz and 160 MHz). It employs knapsack-based scheduling, custom optimization, and the Hungarian algorithm to optimize data rates and minimize delays. The simulation supports Single-Link (SL), Simultaneous Transmit/Receive (STR), and Non-Simultaneous Transmit/Receive (NSTR) devices, with configurable time-critical traffic. Results are exported to CSV files for analysis.

Key components:
- **`ap.h`/`ap.cpp`**: Manage STAs, allocate Multiple Resource Units (MRUs), and simulate transmissions.
- **`scheduler.h`/`scheduler.cpp`**: Generate traffic and schedule access for single- or dual-channel scenarios.
- **`station.h`/`station.cpp`**: Model STA attributes, packet queues, and channel-specific data rates.
- **`packet.h`/`packet.cpp`**: Define packets with size, arrival time, and deadline.
- **`hungarian_optimizer.h`**: Implements the Hungarian algorithm for resource allocation.
- **`secure_matrix.h`**: Provides a secure matrix class using Eigen for cost assignments.
- **`main.cpp`**: Drives the simulation and outputs performance metrics.

## Features

- **OFDMA Resource Allocation**:
  - Supports MRUs from 26-tone to 4x996-tone, with data rates up to 2402 Mbps (MCS 11, 1024-QAM).
  - Dynamically allocates MRUs based on required data rates and channel conditions (MCS_A, MCS_B).
- **Scheduling Algorithms**:
  - **Knapsack-based** (`knaspack_sra`, Methods 0/3): Maximizes throughput.
  - **Hungarian Algorithm** (`Tzu`, Methods 2/5): Optimizes STA-MRU assignments using maximum-weighted matching.
  - **Custom Optimization** (`opt_RCL`, `opt_FGC`, Methods 1/4): Rough and fine-grained resource classification.
- **Multi-Channel Support**:
  - Manages 320 MHz (148 26-tone units) and 160 MHz (74 26-tone units) channels.
  - Supports MLO with SL (20%), STR (40%), and NSTR (40%) devices.
  - Synchronizes channels (EOSYNC/EOASYNC) based on Effective Bandwidth Ratio (EBR).
- **Traffic Generation**:
  - Generates exponential inter-arrival traffic with 4 priority levels.
  - Configurable arrival rates (20–300 Mbps), packet sizes (200–1000 bytes), and deadlines (150–2000 ms).
- **Station Modeling**:
  - Tracks packet queues, expired packets, and channel-specific transmissions.
  - Supports time-critical traffic with dynamic data rate scaling (`alpha`=2.75).
  - Uses randomized MCS indices (fixed at 11 for calculations).
- **Optimization**:
  - Hungarian algorithm (`hungarian_optimizer.h`) for efficient resource allocation.
  - Secure matrix (`secure_matrix.h`) using Eigen for cost matrix operations.
- **Performance Metrics**:
  - Throughput, delay, packet loss rate, and channel utilization.
  - Analytical models for queue length and mathematical delay.
  - Device-specific metrics for SL, STR, and NSTR.
  - CSV output in `實驗結果1ch/` directory.

## Dependencies

- **C++ Compiler**: C++11 or later (e.g., g++).
- **Eigen Library**: Required for `secure_matrix.h` and `hungarian_optimizer.h`.
  - Install via: `sudo apt-get install libeigen3-dev` (Ubuntu) or equivalent.
  - Ensure Eigen headers are in the include path.
- **Standard Libraries**: `<vector>`, `<map>`, `<queue>`, `<chrono>`, `<random>`, `<iostream>`, `<fstream>`, `<cmath>`, `<exception>`, `<algorithm>`, `<limits>`, `<utility>`, `<string>`, `<sstream>`.
- **Custom Headers**:
  - `packet.h`/`packet.cpp`: Define `Packet` class.
  - `station.h`/`station.cpp`: Define `Station` class.
  - `ap.h`/`ap.cpp`: Define `AP` class.
  - `scheduler.h`/`scheduler.cpp`: Define `Scheduler` class.
  - `hungarian_optimizer.h`: Implements Hungarian algorithm.
  - `secure_matrix.h`: Defines `SecureMat` class.
- **No Other External Libraries**: Relies on standard C++ and Eigen.

## Usage

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/tzhsuan/802.11be-MRU-simulation
   cd 802.11be-MRU-simulation
   ```

2. **Install Eigen**:
   On Ubuntu:
   ```bash
   sudo apt-get install libeigen3-dev
   ```
   Ensure Eigen headers are accessible (e.g., `/usr/include/eigen3`).

3. **Compile the Program**:
   Compile all source files:
   ```bash
   g++ -o sim main.cpp ap.cpp scheduler.cpp station.cpp packet.cpp -std=c++11 -I/usr/include/eigen3
   ```
   Adjust the `-I` path if Eigen is installed elsewhere.

4. **Simulation Parameters**:
   - **Simulation Time**: Set via `Scheduler` constructor (in microseconds).
   - **Bandwidth**: 148 (320 MHz), 74 (160 MHz), or combined (148+74) 26-tone units.
   - **Channels**: Single-channel (`schedule_access`) or dual-channel (`schedule_access2CH`).
   - **Scheduling Methods**:
     - `0` or `3`: Knapsack-based scheduling.
     - `1` or `4`: Custom optimization (`opt_RCL`, `opt_FGC`).
     - `2` or `5`: Hungarian algorithm-based (`Tzu`).
   - **Traffic**:
     - Up to 4 priority levels with packet sizes (200–1000 bytes).
     - Arrival rates in Mbps, converted to packets per microsecond.
     - Deadlines in microseconds for time-critical traffic.
   - **Stations**:
     - SL, STR, NSTR devices with channel-specific MCS (randomized 0–11).
     - Maximum data rates calculated based on MCS and bandwidth (e.g., 2402 Mbps for 320 MHz).
     - Dynamic data rate updates with scaling factor (`alpha`).

5. **Run the Simulation**:
   Execute the binary:
   ```bash
   ./sim
   ```
   The `main.cpp` configures:
   - Simulation time: 5 seconds (5M µs).
   - Method: 2 (Tzu, single-channel; adjustable 0–5).
   - Alpha: 2.75.
   - Priorities: 4 levels, 5 STAs each.
   - Traffic: 20, 160, 300, 100 Mbps; packet sizes 200, 1000, 500, 500 bytes; deadlines 150, 200, 1000, 2000 ms.
   - Devices: 20% SL, 40% STR, 40% NSTR.

6. **Output**:
   - Console: STA-specific throughput, delay, packet loss, channel transmissions, and device type.
   - CSV files: In `實驗結果1ch/` (e.g., `Tzu_1CH_ALL_STA_TH.csv`; uncomment `writeData` calls to enable).

7. **Modify Parameters**:
   Edit `main.cpp`:
   - `Method`: 0–2 (single-channel), 3–5 (dual-channel).
   - `alpha`: Data rate scaling.
   - `sim_time`: Duration.
   - `PRI_PEOPLE`, `TRAFFIC_ARRIVAL_RATES`, `MPDU_LENS`, `DEADLINES`: Traffic settings.
   - `SL_STR_NSTR_RATIO`: Device distribution.

## Code Structure

- **`packet.h`/`packet.cpp`**:
  - Define `Packet` with `packetSize` (bits), `arrival_time`, `deadline` (µs), and `canTrans`.
- **`station.h`/`station.cpp`**:
  - Define the `Station` class with attributes like:
    - Packet queue (`packets`), device type (`device`), priority (`priority`), and STA ID (`STA_ID`).
    - Channel-specific MCS arrays (`MCS_A`, `MCS_B`) and data rates (`required_dr_A`, `required_dr_B`).
    - Transmission metrics (`success_trans`, `n_suc_packet_chA`, `n_suc_packet_chB`, `total_dealy_time`).
    - Analytical metrics (`ana_TH`, `ana_D`, `ana_RFs`).
  - Implement methods for:
    - Initializing STAs with randomized MCS indices (`Station` constructor).
    - Updating required data rates (`updateRD`) with time-critical constraints.
    - Tracking expired packets (`updateExpired`).
    - Updating MRU assignments (`updateRMRU`).
    - Sorting STAs by ID or required data rate (`compareBySTAID`, `compareByRD`).
- **`ap.h`/`ap.cpp`**:
  - Define the `AP` class with constants (`MRUs`, `MRUs_dr`, `MPDU_LENS`) and methods for:
    - STA management (`updateSTAs`, `opt_updateSTAs`).
    - Resource allocation (`knaspack_sra`, `Tzu`, `TzuU`, `opt_RCL`, `opt_FGC`).
    - Transmission simulation (`transmit2STAs`, `sim_transmit2STAs`).
    - Analytical calculations (`cal_STAs_ana`).
    - MRU mapping (`MRU_map_26`, `renew_allocation_table`).
  - Support dual-channel MLO with load balancing (`twoChUsersAlloc`).
- **`scheduler.h`/`scheduler.cpp`**:
  - Define the `Scheduler` class for:
    - Traffic generation (`generate_traffic`) with exponential inter-arrival times.
    - Scheduling access for single-channel (`schedule_access`) or dual-channel (`schedule_access2CH`).
    - Managing timing (SIFS: 16 µs, PIFS: 25 µs, ACK: 60 µs, TXOP: 5000 µs).
    - Supporting multiple scheduling algorithms and channel synchronization.
- **`hungarian_optimizer.h`**:
  - Implements Hungarian algorithm for optimal assignments using Munkres steps.
  - Supports maximization/minimization of cost matrices.
- **`secure_matrix.h`**:
  - Defines `SecureMat` using Eigen for dynamic matrix operations with reserved memory.
- **`main.cpp`**:
  - Configures experiments, runs simulations, and computes metrics.
  - Supports variable STA counts and analytical error analysis.

  

## Example Output

Default settings (`Method=2`, `sim_time=5M µs`):
```
alpha = 2.75
Method= 2
minMCS = 11, min channel rate= 2402
優先級別 = 4
STA ID:0, 封包總數 = 1250, success transmission = 1000, 吞吐量 = 0.32, 延遲 = 0.5
封包遺失率 = 0.2, 在A頻道傳輸數量 = 1000, 在B頻道傳輸數量 = 0
Device type = SL
...
優先級別 = 1, 總吞吐量 = 1.6 Mbits
平均320 MHz頻道利用率 = 0.85
```

## Performance Analysis

- **Throughput**: `(success_trans * packet_size * 8) / sim_time` (Mbps).
- **Delay**: `total_dealy_time / (success_trans * 1000)` (ms).
- **Packet Loss**: `n_expired_packet / packets.size()`.
- **Channel Utilization**: Normalized to 2402 Mbps (320 MHz) or 1201 Mbps (160 MHz).
- **Device Metrics**: Throughput and delay for SL, STR, NSTR in dual-channel mode.
- **Analytical Comparison**: Simulated vs. mathematical delay (enable `get_anaErrRate`).

## Notes

- **Assumptions**:
  - MCS indices for data rate calculations.
  - Eigen library is properly installed.
- **Limitations**:
  - CSV output disabled by default (uncomment `writeData`).
  - Memory-intensive due to vector and matrix operations.
- **Extensibility**:
  - Add new scheduling algorithms in `Scheduler` or `AP`.
  - Enhance `Station` with interference models.
  - Extend `HungarianOptimizer` for additional optimization constraints.

## Contributing

Contributions are welcome! Submit issues or pull requests for:
- Bug fixes (e.g., memory leaks).
- New algorithms or traffic models.
- Visualization tools for CSV data.
- Documentation improvements.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
