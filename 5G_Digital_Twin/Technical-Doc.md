 The 5G Digital Twin Engine

---



This architecture depicts a 5G Digital Twin (NDT) system where a "Twin Network" mirrors the "Real Network" to simulate and optimize performance using a Network Data Analytics Function (NWDAF).

The two layers exchange real-time feedback via N7 and N8 interfaces, allowing the PCF (Policy Control Function) to update the live network based on insights gained from the digital simulation.
<img width="782" height="703" alt="Untitled Diagram drawio(8)" src="https://github.com/user-attachments/assets/9db93e08-2be6-4e5b-b04f-8265dc3eaa32" />  


The platform is structured into five layers:
This diagram illustrates the simulation stack for a 5G Digital Twin, mapping low-level C++ bindings (ns-3) to high-level protocol and radio engines for realistic network modeling.

It flows from raw packet and signal physics up through 5G Core Network Functions (like NWDAF and RIC) to a real-time web dashboard for monitoring and API interaction.

<img width="683" height="566" alt="Untitled Diagram drawio(10)" src="https://github.com/user-attachments/assets/ffeb8351-1dee-4d87-bdd4-8109b071b4ad" />

## 1. The Core Ideology: 
The platform operates on the principle of **Coherent Synchronization** between two distinct simulation environments:

* **RealisticSim (The "Real" Network):** A single-cell reference simulator that acts as the "source of truth" for 5G protocol state machines (NAS, RRC). It manages the actual lifecycle of every UE, from initial cell search to PDU session establishment.
* **TwinSim (The Digital Twin):** A multi-gNB environment that mirrors the real network's state but applies complex multi-cell physics. It handles interference from neighboring cells, handovers, and beamforming gains.

The two are kept in sync via a shared **State Bus**, ensuring that if a UE's signal drops in the Digital Twin due to distance, the Protocol Engine in the Real Sim correctly triggers a retransmission or handover.

---

## 2. Protocol Engine & MAC Schedulers

While the physics engine handles the "air," the Protocol Engine builds the actual packets and manages resource allocation.

| Component | Responsibility |
| :--- | :--- |
| **MAC Scheduler** | Decisions on which UE gets Resource Blocks (RBs) every 1ms (PF, Max CQI, or RR). |
| **Network Slicing** | Enforces strict RB limits for **URLLC** (Low Latency) vs **eMBB** (High Bandwidth). |
| **PcapEngine** | Builds binary-compliant frames for Wireshark (NAS, RRC, GTP-U, NGAP). |

### **The Lifecycle of a Packet**
1.  **UE Move**: User drags a UE on the Dashboard.
2.  **Physics Update**: `TwinSim` recalculates distance $\to$ Path Loss $\to$ SINR.
3.  **Link Adaptation**: The `NsLteAmc` module maps SINR to a **CQI** (1–15).
4.  **Resource Mapping**: The Scheduler assigns RBs based on the CQI.
5.  **Packet Construction**: The `Protocol Engine` wraps data in PDCP/RLC/MAC headers.
6.  **Trace Capture**: The `PcapEngine` writes the binary frame to `uu_radio.pcap`.

---

## 3. Software Architecture (ns-3 + Python)

The project utilizes a **Hybrid-Native** approach to ensure high performance without sacrificing ease of use.

* **cppyy Integration**: The system uses `cppyy` to dynamically bind Python to high-performance C++ libraries of **ns-3 (v3.46)**. This allows math to run at native speeds.
* **Fallback Mechanism**: If ns-3 is missing, the system activates a pure-Python port of the 3GPP math, ensuring the simulation runs on any machine.


> You can observe this entire flow by downloading the `.pcap` files from the dashboard and opening them in Wireshark to see the standards-compliant 5G headers.
