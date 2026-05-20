

# Overview

A real-time, physics-accurate simulation platform that mirrors live 5G network behaviour in a virtual digital twin. The system pairs a single-cell real network simulator (RealisticSim) with a fully configurable multi-gNB digital twin (TwinSim), exposes all metrics through a browser dashboard, and generates Wireshark-compatible PCAP traces across all 5G protocol layers.

**Technology Stack:** Python 3.10+ | ns-3 v3.46 | cppyy | 3GPP TR 38.901 | HTTP/REST


# Architecture

<img width="571" height="231" alt="Untitled Diagram drawio(3)" src="https://github.com/user-attachments/assets/bf45f94a-fe01-47b9-8677-2ec266d2471d" />


The platform is structured into five layers:

| Layer | Components | Responsibility |
|---|---|---|
| ns3 Binding Layer | cppyy, _Ns3UmiLossModel | Loads native ns3 C++ libraries; provides Python fallback models |
| Protocol Engine | PcapEngine, Packet builders | Builds standards-compliant binary frames; writes PCAP captures |
| PHY / Radio Engine | NsLteAmc, NsThreeGppUmi, NsLteSpectrumPhy | 3GPP TR 38.901 channel math — SINR, CQI, MCS, TBS, latency |
| Simulation Engine | RealisticSim, TwinSim, UE, GNB, NAS state machine | Runs real and twin simulations; manages UE lifecycle and handovers |
| 5G Core NFs | NWDAF, NEF, NearRTRIC, UPFMonitor, SBABus | Core analytics, exposure, RIC E2 telemetry, UPF monitoring |


# Installation & Setup
## Prerequisites
* **Operating System**: Ubuntu 22.04 or later (recommended).
* **Internet Connection**: Required to download ns-3 source files and system dependencies.
* **Sudo Access**: Required to install networking libraries.

---

##  Quick Start Instructions

Follow these steps to get the dashboard running:

### 1. Clone the Repository
Open your terminal and download the project files :
```bash
git clone https://github.com/mhradhika/5GTrial.git
cd 5GTrial
cd 5G_Digital_Twin
```

### 2. Run the Installer
```bash
chmod +x install.sh
./install.sh
```
On startup you will see:  
[1/3] Starting Real Network simulator ...  
[2/3] Starting Digital Twin (multi-gNB, math-coherent) ...  
[3/3] Dashboard -> http://localhost:9095/  

Open `http://localhost:9095` in your browser.
The dashboard will look like this
<img width="2500" height="750" alt="image" src="https://github.com/user-attachments/assets/65ee8f9c-980e-4b17-b464-b88e3fad82e9" />


**PCAP captures** are saved to `./pcap/` and can be downloaded via the dashboard or REST API:
```bash
curl -O http://localhost:9095/download_pcap?iface=uu_radio
curl -O http://localhost:9095/download_pcap?iface=n2_ngap
```

**Fallback mode (no ns3):** If ns3 libraries are not found, the platform automatically falls back to a pure-Python implementation of all 3GPP TR 38.901 models — no changes needed, just run the same command.

**Stop the application:**
```bash
# Ctrl+C in terminal, or:
kill -SIGINT <pid>
```

---

# Protocol & Radio Engine

## PCAP Capture & Packet Builders

The protocol engine generates standards-compliant binary packets for all major 5G interfaces, openable directly in Wireshark.

**Seven PCAP interfaces:**

| Interface | Description |
|---|---|
| `uu_radio` | Uu radio — PHY DCI, MAC PDU, RLC, PDCP, RRC |
| `n2_ngap` | N2 — gNB to AMF NGAP messages over SCTP (port 38412) |
| `n3_gtpu` | N3 — gNB to UPF GTP-U user-plane tunnel (UDP port 2152) |
| `f1_du_cu` | F1 — DU to CU F1AP messages (UDP port 38472) |
| `sba_http2` | SBA — NF-to-NF HTTP/2 (nudm, nsmf, namf, nwdaf) |
| `coap_mmtc` | CoAP — mMTC IoT sensor data |
| `icmp_ctrl` | ICMP ping and ARP — UE connectivity checks |

**Packet builders implemented:** Ethernet (L2), IPv4 (L3), UDP/SCTP (L4), GTP-U tunnel, 5G NAS (11 message types), NGAP (14 procedures), RRC (14 message types), PDCP (18-bit SN, HMAC-SHA256 MAC-I), RLC AM, MAC PDU, PHY DCI, F1AP (8 procedures), HTTP/2 (10 frame types), CoAP, ICMP, ARP.

**Event sequences generated per procedure:**
- **NAS Registration:** RRC Setup Request → NAS Registration Request → NGAP Initial UE Message
- **Authentication:** NAS Auth Request/Response → NGAP UL NAS Transport → SBA nudm-ueau
- **RRC Setup:** RRC Setup/Complete → Security Mode Command/Complete → F1AP DL RRC Transfer
- **PDU Session:** NAS PDU Est. Req/Resp → NGAP PDU Session Setup → GTP-U tunnel → ARP → SBA nsmf
- **Data plane:** Per-tick GTP-U user-plane records per active UE (eMBB/URLLC/mMTC traffic)
- **Handover:** NGAP Handover Required/Request/Notify → RRC Reconfiguration

## PHY / AMC / Propagation Models

The PHY engine implements the full 3GPP TR 38.901 UMi Street Canyon channel model chain.

**Path loss (3GPP TR 38.901 Table 7.4.1-1):**
- LOS (dual-slope): `PL = 32.4 + 21·log10(d3D) + 20·log10(fc)` below breakpoint; steeper slope above
- NLoS: `PL = max(PL_LOS, 35.3·log10(d3D) + 22.4 + 21.3·log10(fc) - 0.3·(hUT - 1.5))`
- LOS probability: `P_LOS = min(18/d, 1.0) · (1 - exp(-d/36)) + exp(-d/36)`

**Shadow fading (AR(1) log-normal):**
- σ = 4.0 dB (LOS), 7.82 dB (NLoS); decorrelation distance = 10 m
- Update: `s_new = α·s_prev + √(1-α²)·σ·N(0,1)`, where `α = exp(-Δd/10)`

**Fast fading:** Rician (LOS, K = 9 dB) or Rayleigh (NLoS); Doppler = `v·fc/c`

**SINR pipeline:**
SINR = Signal / (Noise + Inter-cell interference + Intra-cell interference)
where Signal includes path loss + 64-antenna beamforming gain (up to 18 dB) + 4-antenna MRC gain (6 dB) + shadow + fast fading.

**CQI → MCS → TBS chain (3GPP TS 36.213):**
- CQI 0–15 mapped to QPSK / 16QAM / 64QAM / 256QAM
- BLER computed via logistic curve with 4 dB OLMA margin and HARQ combining gain (3 dB/round)

## MAC Schedulers & Latency

**Three schedulers available (switchable live via dashboard):**

| Scheduler | Algorithm | Best For |
|---|---|---|
| Proportional Fair (PF) | Sort UEs by instant_rate / avg_rate; EMA α=0.1 | Fairness + efficiency (default) |
| Round Robin (RR) | Equal RB allocation per UE | Pure fairness |
| Max CQI | Sort by descending CQI | Maximum aggregate throughput |

**Resource block slicing (default, per gNB):**
- eMBB: 216 RBs (80%)
- URLLC: 27 RBs (10%)
- mMTC: 27 RBs (10%)

RB allocation is tunable per-gNB at runtime via `/tweak_rb` REST endpoint.

**Throughput formula:**
Tput = Qm × CR × 12 × 14 × N_layers × N_RB × OFDM_eff × HARQ_eff × PDCCH_prob × SPS / 1e6  (Mbps)

**End-to-end latency model (NsLteLatencyModel):**

| Component | Value |
|---|---|
| TTI | slot_ms × 2 (URLLC) or × 4 (eMBB/mMTC) |
| Processing | (UE_PROC=3 + gNB_PROC=3) × slot_ms |
| Propagation | dist_m / c × 1000 ms |
| Core delay | 0.3 ms (URLLC), 1.0 ms (others) |
| HARQ retransmission | BLER × 8-slot RTT |
| Queuing | M/D/1 model; URLLC capped 1–4 ms, eMBB bounded 2–50 ms |
