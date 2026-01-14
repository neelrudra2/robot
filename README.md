<h1 align="center">🤖 Multi-Robot Warehouse Automation Simulator</h1>

<p align="center">
  <i>A Python-based simulation of autonomous warehouse robots performing dynamic task allocation, path planning, and collision-free navigation.</i>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white">
  <img src="https://img.shields.io/badge/Path%20Planning-A*-orange">
  <img src="https://img.shields.io/badge/Robotics-Simulation-blue">
  <img src="https://img.shields.io/badge/Status-Completed-success">
</p>

---

## 🚀 Project Motivation

Modern warehouses rely on **multiple autonomous robots** to efficiently move goods while avoiding congestion and collisions.  
This project simulates a **multi-robot warehouse environment**, focusing on **dynamic task allocation**, **optimal path planning**, and **real-time re-planning** under shared-space constraints.

---

## 🧩 Project Overview

The **Multi-Robot Warehouse Automation Simulator** models **5–10 autonomous robots** operating inside a **grid-based warehouse**.  
Robots dynamically select parcels, compute optimal routes using **A\* pathfinding**, and re-plan paths in real time to ensure **collision-free navigation**.

The simulation emphasizes **algorithmic efficiency and coordination**, rather than hardware or ML-based control.

---

## 🎯 Key Achievements

- Simulated **5–10 autonomous robots** performing parcel pickup and delivery tasks  
- Implemented **A\* pathfinding**, reducing average travel distance by **25–30%** compared to naive routing  
- Achieved **collision-free navigation** using dynamic route re-planning  
- Implemented **dynamic task allocation**, preventing multiple robots from targeting the same parcel  

---

## 🖼️ Simulation Visuals

<p align="center">
  <img src="assets/warehouse-grid.png" width="45%">
  &nbsp;&nbsp;&nbsp;
  <img src="assets/robot-paths.png" width="45%">
</p>

<p align="center">
  <b>Left:</b> Grid-Based Warehouse Environment &nbsp;&nbsp; | &nbsp;&nbsp;
  <b>Right:</b> Robot Paths with A* Planning
</p>

---

## ⚙️ Core Concepts Implemented

| Concept | Description |
|------|------------|
| A* Algorithm | Optimal shortest-path planning using heuristic search |
| Dynamic Task Allocation | Nearest unassigned parcel selection |
| Multi-Agent Coordination | Shared workspace with multiple robots |
| Collision Avoidance | Conflict resolution and path re-planning |
| State-Based Control | Robot states: IDLE, TO_PICKUP, TO_DEST |

---

## 🧠 Robot State Model

Each robot operates using a **finite-state logic**:

| State | Behavior |
|----|--------|
| IDLE | Searches for nearest available parcel |
| TO_PICKUP | Navigates to parcel location |
| TO_DEST | Delivers parcel to destination bin |

Robots dynamically switch states based on task completion and path availability.

---

## 🧵 Path Planning & Re-Planning Strategy

- **A\*** is used for initial route computation  
- Robots treat other robots as **dynamic obstacles**  
- If a path becomes blocked, **re-planning is triggered automatically**  
- Simple conflict resolution ensures only one robot enters a grid cell at a time  

This guarantees **deadlock-free and collision-free movement**.

---

## 📊 Performance Highlights

| Metric | Result |
|-----|------|
| Robots Simulated | 5–10 |
| Path Length Reduction | 25–30% |
| Navigation | Collision-Free |
| Task Assignment | Dynamic & Non-Conflicting |

---

## 🎥 Live Simulation Output

<p align="center">
  <img src="assets/live-simulation.png" width="70%">
</p>

<p align="center">
  <b>Animated visualization of robots picking up and delivering parcels in real time</b>
</p>

---

## ▶️ How to Run

1. Clone the repository  
2. Install dependencies:
  ```bash
  pip install matplotlib numpy
  ```
3. Run the simulator:
   ```bash
   python simulator.py
   ```
4. Observe real-time robot movement and task execution

---

## 🛠️ Tech Stack

1. Python 3.x
2. Matplotlib (Visualization & Animation)
3. A* Pathfinding Algorithm
4. Dynamic Task Allocation
5. Git & GitHub

---

## 🌍 Applications

1. Warehouse automation systems
2. Autonomous mobile robots (AMRs)
3. Multi-agent coordination research
4. Logistics and supply chain simulation
5. Robotics and AI education
