# 🚄 Virtual Coupling Railway Simulation Tool  
### A Safe and Robust Control System Architecture for Virtual Coupling

[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Conference](https://img.shields.io/badge/Presented%20at-WCHR%202025-blue)](https://uic.org/events/world-congress-on-high-speed-rail)


---

## 📘 Overview

This repository provides the official simulation framework and validation environment for the research presented in:

> **A Safe and Robust Control System Architecture for Virtual Coupling**  
> *Mario Terlizzi, Luigi Glielmo, Davide Liuzza*  
> 🏆 *Awarded in collaboration with* [**Rete Ferroviaria Italiana (RFI)**](https://www.rfi.it)  
> 🎤 *Presented at the* [**World Congress on High-Speed Rail 2025**](https://uic.org/events/world-congress-on-high-speed-rail), July 2025  

The tool supports the development, testing, and safety validation of a next-generation control architecture that facilitates the transition from ETCS Level 3 to Virtual Coupling (VC) operations — a key concept for ETCS Level 4.

---

## ✨ Features

✅ Modular simulation framework for two-train VC scenarios  
✅ Full support for parametric uncertainty and non-ideal communication  
✅ Certified safety via **Control Barrier Functions (CBFs)**  
✅ Emergency and cruise control strategies for robust train platooning  
✅ Real-time **Delay Estimator Module**  
✅ Extendable for future research in multi-train convoys and hybrid control  

---

## 🧠 System Architecture

This tool implements a composable, safety-centric architecture with the following key blocks:

<details>
<summary><strong>🧩 Leader RLP Predictor</strong></summary>

Estimates the **Robust Lower Proxy (RLP)** of the leader’s trajectory under worst-case (emergency braking) assumptions.
Ensures the follower can predict a minimum safe trajectory in the absence of fresh data.
</details>

<details>
<summary><strong>🛡️ Safety Control Block</strong></summary>

Implements a **Control Barrier Function**-based safety supervisor.  
Triggers **emergency braking** of the follower if predicted trajectories violate safety guarantees.
</details>

<details>
<summary><strong>🚦 Cruise Virtual Coupling Block</strong></summary>

Predictive controller that regulates the follower’s position during normal operations, optimizing speed tracking and spacing with real-time adaptation based on communication health.
</details>

<details>
<summary><strong>⏱️ Delay Estimator Block (Optional)</strong></summary>

Estimates packet delay distributions and updates a safety horizon dynamically.  
Mitigates unnecessary braking due to transient communication issues.
</details>

---

## 📂 Repository Structure

virtual-coupling-tool/
├── src/
│ ├── controller/ # Control logic (cruise, emergency, switching)
│ ├── dynamics/ # Train longitudinal models + uncertainties
│ ├── safety/ # CBF computation and safe set definitions
│ ├── communication/ # Delay modeling and estimator
│ └── simulation/ # Scenario definitions and run scripts
├── data/ # Train and line parameter sets
├── results/ # Output logs and plots
├── docs/ # Methodology documentation and architecture diagrams
├── tests/ # Unit tests

## 📈 Use Cases

This simulation tool is intended for advanced experimentation and scientific validation in the following domains:

- 📊 **Academic research** in train control and railway safety
- 🚄 **Benchmarking** of virtual coupling (VC) control algorithms
- 🛠️ **Prototyping of ETCS Level 4** control architectures
- 📡 **Validation of delay-tolerant controllers** under communication uncertainty
- 🧮 **Safety enforcement via Control Barrier Functions (CBFs)**

---

## 📖 Citation

If you use this repository in your research or publications, please cite the original work:

```bibtex
@inproceedings{terlizzi2025safe,
  title={A Safe and Robust Control System Architecture for Virtual Coupling},
  author={Terlizzi, Mario and Glielmo, Luigi and Liuzza, Davide},
  booktitle={Proceedings of the World Congress on High-Speed Rail},
  year={2025},
  note={Presented in collaboration with RFI S.p.A.}
}
