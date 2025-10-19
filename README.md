# 🦾 Multi-Robot Warehouse Automation Simulator

A Python-based simulation of autonomous warehouse robots performing **parcel pickup, routing, and delivery** using **A\*** pathfinding, dynamic task allocation, and real-time route replanning.  
This project models the core logic behind **robotic warehouse automation systems** like those developed by companies such as **Unbox Robotics** and **GreyOrange**.

---

## 🚀 Overview

This simulator demonstrates how multiple robots can collaboratively execute delivery tasks within a warehouse grid.  
Each robot autonomously:
- Finds the nearest parcel to pick up.
- Plans an optimal route using **A\*** pathfinding.
- Avoids collisions and dynamically replans when paths are blocked.
- Delivers parcels to their assigned destination bins.

The system runs entirely in software — **no hardware required** — and visualizes robot movement and task execution using **Matplotlib animations**.

---

## ⚙️ Features

✅ Grid-based warehouse layout (10×10 default)  
✅ Multiple robots working in parallel  
✅ A\* algorithm for optimal route planning  
✅ Dynamic task allocation (nearest-task strategy)  
✅ Replanning and obstacle avoidance  
✅ Real-time visualization and stats display  
✅ Logging of robot actions (pickup/delivery events)

---

## 🧠 Concepts Demonstrated

- Multi-Agent Systems  
- Autonomous Navigation  
- Pathfinding (A\*)  
- Swarm Intelligence  
- Task Scheduling & Optimization  
- Warehouse Automation Simulation  

---

## 🧰 Tools & Technologies Used

| Category | Tools / Technologies |
|-----------|----------------------|
| **Programming Language** | Python |
| **Libraries** | Matplotlib, NumPy, Dataclasses |
| **Algorithms** | A\* Pathfinding, Dynamic Task Allocation |
| **Concepts** | Multi-Agent Coordination, Route Optimization, Collision Avoidance |
git clone https://github.com/<your-username>/multi-robot-warehouse-simulator.git
cd multi-robot-warehouse-simulator
