\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
\
   

import math, random, time, threading, json, signal, sys, collections
import struct, os, io, hashlib, ipaddress
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

 

HAS_NS3_PROPAGATION = False
HAS_NS3_ANTENNA     = False                                                        
HAS_NS3_LTE         = False

                                                                               
_Ns3AntennaModel = None
_Ns3UPA          = None
_Ns3MatrixCh     = None

                                                                               
# ---------------------------------------------------------------------------
# NS-3 / cppyy integration
#
# ROOT CAUSE OF ALL PREVIOUS CRASHES:
#   cppyy's Cling/ROOT type-reflection machinery uses process-global state.
#   Calling ANY cppyy-wrapped C++ from a thread other than the one that
#   initialised cppyy causes segfaults inside TListOfEnums::Load() —
#   completely independent of Python's GIL or any Python-level lock.
#
# SOLUTION — main-thread NS-3 dispatcher:
#   Background sim threads never touch cppyy objects directly.  Instead they
#   post a callable to _NS3_QUEUE and block on a threading.Event until the
#   main thread (inside its server.handle_request() loop) drains the queue
#   and executes each callable, returning the result via the same event.
#
#   _ns3_call(fn) — call fn() on the main thread, return its result.
#   _ns3_drain()  — drain the queue once; called from the main loop.
# ---------------------------------------------------------------------------

import queue as _queue

_NS3_QUEUE = _queue.Queue()   # items: (fn, result_list, event)

def _ns3_call(fn):
    """
    Execute fn() on the main thread and return its result.
    Safe to call from any background thread.
    If we are already on the main thread (e.g. during startup) just call directly.
    """
    import threading as _th
    if _th.current_thread() is _th.main_thread():
        return fn()
    result = [None]
    exc    = [None]
    done   = _th.Event()
    def _wrapper():
        try:    result[0] = fn()
        except Exception as e: exc[0] = e
        finally: done.set()
    _NS3_QUEUE.put(_wrapper)
    done.wait()
    if exc[0] is not None:
        raise exc[0]
    return result[0]

def _ns3_drain():
    """Drain pending NS-3 calls from the main thread. Call this in the server loop."""
    try:
        while True:
            fn = _NS3_QUEUE.get_nowait()
            fn()
    except _queue.Empty:
        pass

# ---------------------------------------------------------------------------
# Load NS-3 libraries (main thread, at import time — safe)
# ---------------------------------------------------------------------------

try:
    import cppyy as _cppyy

    _NS3_LIB = "/home/cleanuser/ns-3-dev/build/lib"
    _NS3_INC = "/home/cleanuser/ns-3-dev/build/include"

    if not os.path.isdir(_NS3_LIB):
        raise ImportError(f"NS-3 lib dir not found: {_NS3_LIB}")

    _cppyy.add_library_path(_NS3_LIB)
    _cppyy.add_include_path(_NS3_INC)

    for _lib in [
        "libns3.46-core-debug",
        "libns3.46-network-debug",
        "libns3.46-mobility-debug",
        "libns3.46-antenna-debug",
        "libns3.46-propagation-debug",
        "libns3.46-spectrum-debug",
        "libns3.46-lte-debug",
    ]:
        _cppyy.load_library(_lib)

    _cppyy.include("ns3/core-module.h")
    _cppyy.include("ns3/mobility-module.h")
    _cppyy.include("ns3/three-gpp-propagation-loss-model.h")
    _cppyy.include("ns3/three-gpp-antenna-model.h")
    _cppyy.include("ns3/lte-amc.h")

    _ns3 = _cppyy.gbl.ns3

    # --- helper: create an NS-3 object and immediately pin its ownership so
    #     Python's GC never calls ns3::Object::DoDelete() on it.
    #     Must be called from the main thread only (via _ns3_call if needed).
    def _ns3_create_object(type_name):
        cls = getattr(_ns3, type_name)
        obj = _ns3.CreateObject[cls]()
        # Try set_ownership if available (newer cppyy builds).
        # On older builds it isn't exposed — in that case we keep a permanent
        # reference in _NS3_PINNED so the Python wrapper is never GC'd.
        try:
            import cppyy.ll as _ll
            _ll.set_ownership(obj, False)
        except (ImportError, AttributeError):
            _NS3_PINNED.append(obj)   # permanent ref — prevents GC / DoDelete
        return obj

    _NS3_PINNED = []   # strong refs to all NS-3 objects; prevents GC-triggered DoDelete

    def _Ns3UmiLossModel():
        obj = _ns3.CreateObject[_ns3.ThreeGppUmiStreetCanyonPropagationLossModel]()
        try:
            import cppyy.ll as _ll
            _ll.set_ownership(obj, False)
        except (ImportError, AttributeError):
            _NS3_PINNED.append(obj)
        return obj

    _Ns3MobilityHelper = _ns3.MobilityHelper
    _Ns3PosModel       = _ns3.ConstantPositionMobilityModel
    _Ns3DoubleValue    = _ns3.DoubleValue
    _Ns3Vector         = _ns3.Vector

    HAS_NS3_PROPAGATION = True
    print("[INFO] ns3.propagation bound — using native 3GPP TR 38.901 path-loss math.")

    try:
        _lteamc_obj = _ns3.CreateObject[_ns3.LteAmc]()
        try:
            import cppyy.ll as _ll
            _ll.set_ownership(_lteamc_obj, False)
        except (ImportError, AttributeError):
            _NS3_PINNED.append(_lteamc_obj)

        class _Ns3LteAmc:
            @staticmethod
            def GetCqiFromSpectralEfficiency(se: float) -> int:
                return int(_lteamc_obj.GetCqiFromSpectralEfficiency(se))
            @staticmethod
            def GetMcsFromCqi(cqi: int) -> int:
                return int(_lteamc_obj.GetMcsFromCqi(cqi))
            @staticmethod
            def GetTbSizeFromMcsNprb(mcs: int, nprb: int) -> int:
                return int(_lteamc_obj.GetTbSizeFromMcsNprb(mcs, nprb))
            @staticmethod
            def GetDlTbSizeFromMcs(mcs: int, nrb: int) -> int:
                return int(_lteamc_obj.GetDlTbSizeFromMcs(mcs, nrb))
            @staticmethod
            def GetUlTbSizeFromMcs(mcs: int, nrb: int) -> int:
                return int(_lteamc_obj.GetUlTbSizeFromMcs(mcs, nrb))

        HAS_NS3_LTE = True
        print("[INFO] ns3.lte (LteAmc) bound — using native AMC lookup tables.")

    except Exception as _lte_e:
        print(f"[WARN] ns3.lte not found — falling back to Python AMC engine. ({_lte_e})")
        _Ns3LteAmc = None

except Exception as _prop_e:
    print(f"[WARN] ns3.propagation not found — falling back to Python path-loss engine. ({_prop_e})")
    print(f"[WARN] ns3.lte not found — falling back to Python AMC engine.")
    def _Ns3UmiLossModel():       return None
    def _ns3_create_object(t):    return None
    _Ns3MobilityHelper = None
    _Ns3PosModel       = None
    _Ns3DoubleValue    = None
    _Ns3Vector         = None
    _Ns3LteAmc         = None
    _NS3_PINNED        = []

                                                                     
                                                           
                                                                     
                                                               

PCAP_DIR        = "./pcap"
PCAP_RING_MAX   = 10_000                                          
_PCAP_LOCK      = threading.Lock()

                                                               
_PCAP_MAGIC     = 0xa1b2c3d4                                             
_PCAP_VER_MAJ   = 2
_PCAP_VER_MIN   = 4
_PCAP_SNAPLEN   = 65535
DLT_EN10MB      = 1                                                  
DLT_USER0       = 147                                                         

                                                                
PCAP_IFACES = {
    "uu_radio":   {"dlt": DLT_USER0,  "desc": "Uu radio interface (PHY/MAC/RLC/PDCP/RRC)"},
    "n2_ngap":    {"dlt": DLT_EN10MB, "desc": "N2 NGAP — gNB↔AMF control plane"},
    "n3_gtpu":    {"dlt": DLT_EN10MB, "desc": "N3 GTP-U — gNB↔UPF user plane"},
    "f1_du_cu":   {"dlt": DLT_EN10MB, "desc": "F1-U/F1-C DU↔CU interface"},
    "sba_http2":  {"dlt": DLT_EN10MB, "desc": "SBA HTTP/2 service mesh (N11/N12/N15)"},
    "coap_mmtc":  {"dlt": DLT_EN10MB, "desc": "CoAP mMTC application (UE→IoT server)"},
    "icmp_ctrl":  {"dlt": DLT_EN10MB, "desc": "ICMP/ARP UE connectivity checks"},
}

def _pcap_global_header(dlt):
    """libpcap global header (24 bytes)."""
    return struct.pack("<IHHiIII",
        _PCAP_MAGIC, _PCAP_VER_MAJ, _PCAP_VER_MIN,
        0,                            
        0,                     
        _PCAP_SNAPLEN,
        dlt,
    )

def _pcap_record(ts_us, data):
    """libpcap packet record header + data."""
    sec  = ts_us // 1_000_000
    usec = ts_us  % 1_000_000
    n    = len(data)
    hdr  = struct.pack("<IIII", sec, usec, n, n)
    return hdr + data

                                                               
class PcapIface:
    """
    NS3-inspired per-interface PCAP writer.
    Keeps a deque ring buffer in memory (fast), flushes to disk on demand.
    """
    __slots__ = ("name","dlt","desc","_ring","_counter","_total","_lock")

    def __init__(self, name, dlt, desc=""):
        self.name    = name
        self.dlt     = dlt
        self.desc    = desc
        self._ring   = collections.deque(maxlen=PCAP_RING_MAX)
        self._counter= 0
        self._total  = 0
        self._lock   = threading.Lock()

    def write(self, data: bytes):
        ts_us = int(time.time() * 1e6)
        rec   = _pcap_record(ts_us, data)
        with self._lock:
            self._ring.append(rec)
            self._counter += 1
            self._total   += 1

    def to_bytes(self) -> bytes:
        """Serialize full pcap file (global header + all buffered records)."""
        with self._lock:
            records = list(self._ring)
        buf = io.BytesIO()
        buf.write(_pcap_global_header(self.dlt))
        for r in records:
            buf.write(r)
        return buf.getvalue()

    def flush_to_disk(self):
        os.makedirs(PCAP_DIR, exist_ok=True)
        path = os.path.join(PCAP_DIR, f"{self.name}.pcap")
        with open(path, "wb") as f:
            f.write(self.to_bytes())

    def stats(self):
        with self._lock:
            return {"buffered": len(self._ring), "total": self._total}

class PcapEngine:
    """
    Central PCAP engine — one PcapIface per logical 5G interface.
    Matches NS3's per-node per-interface .pcap file naming.
    """
    def __init__(self):
        self.ifaces = {
            name: PcapIface(name, cfg["dlt"], cfg["desc"])
            for name, cfg in PCAP_IFACES.items()
        }
        os.makedirs(PCAP_DIR, exist_ok=True)

    def write(self, iface: str, pkt: bytes):
        if iface in self.ifaces:
            self.ifaces[iface].write(pkt)

    def download(self, iface: str) -> bytes:
        if iface in self.ifaces:
            return self.ifaces[iface].to_bytes()
        return b""

    def flush_all(self):
        for ifc in self.ifaces.values():
            ifc.flush_to_disk()

    def stats(self):
        return {name: ifc.stats() for name, ifc in self.ifaces.items()}

pcap_engine = PcapEngine()

                                                                

def _u8(v):  return struct.pack("B", v & 0xFF)
def _u16be(v): return struct.pack(">H", v & 0xFFFF)
def _u32be(v): return struct.pack(">I", v & 0xFFFFFFFF)
def _u16le(v): return struct.pack("<H", v & 0xFFFF)
def _u32le(v): return struct.pack("<I", v & 0xFFFFFFFF)

                                                                
def eth_frame(src_mac, dst_mac, ethertype, payload):
    """Standard 14-byte Ethernet II header."""
    return bytes.fromhex(dst_mac.replace(":","")) +\
           bytes.fromhex(src_mac.replace(":","")) +\
           struct.pack(">H", ethertype) + payload

def _ue_mac(ue_id):
    return f"02:00:00:00:{(ue_id>>8)&0xFF:02x}:{ue_id&0xFF:02x}"

def _gnb_mac(gnb_id):
    return f"02:00:01:00:{(gnb_id>>8)&0xFF:02x}:{gnb_id&0xFF:02x}"

                                                                
def ipv4_pkt(src_ip, dst_ip, proto, payload, tos=0, ttl=64):
    """Minimal IPv4 header, no options. Computes real checksum."""
    ihl    = 5
    length = 20 + len(payload)
    ip_id  = random.randint(0, 0xFFFF)
    hdr_no_csum = struct.pack(">BBHHHBBH4s4s",
        (4<<4)|ihl, tos, length, ip_id, 0x4000,
        ttl, proto, 0,
        ipaddress.IPv4Address(src_ip).packed,
        ipaddress.IPv4Address(dst_ip).packed,
    )
    words  = struct.unpack(">10H", hdr_no_csum)
    csum   = sum(words); csum = (csum >> 16) + (csum & 0xFFFF)
    csum   = ~(csum + (csum>>16)) & 0xFFFF
    hdr    = hdr_no_csum[:10] + struct.pack(">H", csum) + hdr_no_csum[12:]
    return hdr + payload

def udp_seg(src_port, dst_port, payload):
    """UDP datagram (checksum=0 allowed per RFC 768)."""
    length = 8 + len(payload)
    return struct.pack(">HHHH", src_port, dst_port, length, 0) + payload

def _sctp_chunk(chunk_type, flags, data):
    """Minimal SCTP chunk."""
    length = 4 + len(data)
    return struct.pack(">BBH", chunk_type, flags, length) + data

def sctp_pkt(src_port, dst_port, vtag, chunks):
    """SCTP common header + one or more chunks."""
    hdr = struct.pack(">HHI", src_port, dst_port, vtag)
                                                                 
    hdr += struct.pack(">I", 0)
    return hdr + chunks

                                                                
def gtpu_pkt(teid, payload):
    """GTP-U v1 G-PDU header."""
    flags   = 0x30                         
    msgtype = 0xFF               
    length  = len(payload)
    return struct.pack(">BBHI", flags, msgtype, length, teid) + payload

                                                               
_NGAP_PROCEDURES = {
    "InitialUEMessage":     0x0F,
    "InitialContextSetup":  0x0E,
    "UEContextRelease":     0x2A,
    "PathSwitchRequest":    0x2B,
    "PDUSessionSetupReq":   0x1D,
    "PDUSessionSetupResp":  0x1E,
    "NGSetup":              0x15,
    "Paging":               0x14,
    "UplinkNASTransport":   0x2E,
    "DownlinkNASTransport": 0x11,
    "HandoverRequired":     0x00,
    "HandoverRequest":      0x01,
    "HandoverNotify":       0x02,
    "RerouteNASRequest":    0x26,
}

def ngap_pdu(procedure, criticality=0, ue_id=0, gnb_id=0, extra=b""):
    """
    Minimal NGAP PDU:
      initiatingMessage (tag=0x00) | successfulOutcome (0x20) | unsuccessfulOutcome (0x40)
      procedureCode, criticality, value (TLV)
    Mirrors NS3 nr-lte-helper NGAP framing for Wireshark dissection.
    """
    proc_code = _NGAP_PROCEDURES.get(procedure, 0xFF)
    value_ie  = struct.pack(">HH", ue_id & 0xFFFF, gnb_id & 0xFFFF) + extra
                                                          
    value_seq = b"\x30" + bytes([len(value_ie)]) + value_ie
    pdu       = struct.pack(">BB", 0x00, proc_code) +\
                struct.pack(">B", criticality) +\
                struct.pack(">H", len(value_seq)) + value_seq
    return pdu

                                                               
_NAS_MSG_TYPES = {
    "RegistrationRequest":    0x41,
    "RegistrationAccept":     0x42,
    "AuthenticationRequest":  0x56,
    "AuthenticationResponse": 0x57,
    "SecurityModeCommand":    0x5D,
    "SecurityModeComplete":   0x5E,
    "PDUSessionEstReq":       0xC1,
    "PDUSessionEstAcc":       0xC2,
    "DeregistrationRequest":  0x45,
    "ServiceRequest":         0x4C,
    "ConfigurationUpdate":    0x54,
}

def nas_pdu(msg_type, ue_id=0, payload=b""):
    """
    5GS NAS PDU:
      [Extended protocol discriminator=0x7E][Security header=0x00]
      [Message type][UE id 2B][payload]
    """
    msg_code = _NAS_MSG_TYPES.get(msg_type, 0xFF)
    return struct.pack(">BBB", 0x7E, 0x00, msg_code) +\
           struct.pack(">H", ue_id & 0xFFFF) + payload

                                                               
_RRC_MSG_TYPES = {
    "RrcSetupRequest":          0x01,
    "RrcSetup":                 0x02,
    "RrcSetupComplete":         0x03,
    "SecurityModeCommand":      0x04,
    "SecurityModeComplete":     0x05,
    "RrcReconfiguration":       0x06,
    "RrcReconfigurationComplete":0x07,
    "MeasurementReport":        0x08,
    "RrcRelease":               0x09,
    "RrcReestablishmentRequest":0x0A,
    "UlInformationTransfer":    0x0B,
    "DlInformationTransfer":    0x0C,
    "Paging":                   0x0D,
    "SystemInformationBlockType1":0x0E,
}

def rrc_pdu(msg_type, ue_id=0, gnb_id=0, payload=b""):
    """Minimal RRC PDU with type byte + UE/gNB ids."""
    msg_code = _RRC_MSG_TYPES.get(msg_type, 0xFF)
    return struct.pack(">BBHH", msg_code, 0x00, ue_id & 0xFFFF, gnb_id & 0xFFFF) + payload

                                                                
def pdcp_pdu(bearer_id, sn, payload, integrity_tag=None):
    """
    PDCP data PDU (SRB/DRB).
    SN = 12 bits (SRB) or 18 bits (DRB long).
    Includes simulated 4-byte MAC-I integrity tag (truncated HMAC-SHA256).
    """
                                                
    dc_bearer_sn = (1 << 23) | ((bearer_id & 0xF) << 18) | (sn & 0x3FFFF)
    hdr = struct.pack(">I", dc_bearer_sn)[1:]           
    if integrity_tag is None:
                                                          
        integrity_tag = hashlib.sha256(payload).digest()[:4]
    return hdr + payload + integrity_tag

                                                                
def rlc_am_pdu(sn, payload, poll=False, seg_offset=None):
    """
    RLC AM data PDU — 18-bit SN format.
    D/C=1, P=poll, SI=last_seg, SN[18], [SO if seg], data
    """
    si = 0b00            
    p  = 1 if poll else 0
    hdr_word = (1 << 23) | (p << 22) | (si << 20) | (sn & 0x3FFFF)
    return struct.pack(">I", hdr_word)[1:] + payload                 

                                                                
_MAC_LCID_DL  = {"CCCH":0,"DCCH":1,"DTCH_eMBB":4,"DTCH_URLLC":5,"DTCH_mMTC":6,
                 "BSR_short":0x3D,"BSR_long":0x3E,"Padding":0x3F}

def mac_pdu(lcid, payload, with_bsr=False):
    """
    MAC PDU: subheader [R|F|LCID] + optional L + SDU.
    F=1 → 2-byte length field; F=0 → 1-byte length.
    Optional short BSR CE before SDU.
    """
    out = b""
    if with_bsr:
                                                            
        bsr_ce = struct.pack("B", (1 << 6) | 0x25)
        out   += struct.pack("B", 0x3D) + bsr_ce                        
    L = len(payload)
    if L <= 127:
        out += struct.pack(">BB", lcid & 0x3F, L)
    else:
        out += struct.pack(">BH", (1<<6)|(lcid & 0x3F), L)
    out += payload
                                             
    pad = (4 - len(out) % 4) % 4
    if pad:
        out += b"\x3f" + bytes(pad - 1)
    return out

                                                                
def phy_dci_frame(ue_id, gnb_id, alloc_rb, mcs, layers, tbs_bytes, direction="DL"):
    """
    NS3-style PHY frame: DCI + transport-block header.
    Format inspired by ns3::NrPhySap::MacToPhyDataInfo.
    direction: "DL" (PDSCH) or "UL" (PUSCH)
    """
    dir_byte = 0x00 if direction == "DL" else 0x01
                                             
    dci = struct.pack(">BBHHBBH",
        dir_byte,
        mcs & 0x1F,                            
        ue_id & 0xFFFF,
        gnb_id & 0xFFFF,
        alloc_rb & 0xFF,                                              
        layers & 0x07,                        
        tbs_bytes & 0xFFFF,                              
    )
                                                                    
    tb_payload = bytes([random.randint(0,255) for _ in range(min(tbs_bytes, 64))])
    return dci + tb_payload

                                                                
_F1AP_PROC = {
    "F1Setup":              0x00,
    "UEContextSetup":       0x05,
    "UEContextRelease":     0x06,
    "DLRRCMessageTransfer": 0x09,
    "ULRRCMessageTransfer": 0x0A,
    "InitialULRRCTransfer": 0x0B,
    "Paging":               0x04,
    "GNBDUConfigUpdate":    0x03,
}

def f1ap_pdu(procedure, du_id=0, cu_id=0, ue_id=0, payload=b""):
    proc_code = _F1AP_PROC.get(procedure, 0xFF)
    return struct.pack(">BHHH", proc_code, du_id&0xFFFF, cu_id&0xFFFF,
                       ue_id&0xFFFF) + payload

                                                                
_HTTP2_TYPES = {"DATA":0,"HEADERS":1,"PRIORITY":2,"RST_STREAM":3,"SETTINGS":4,
                "PUSH_PROMISE":5,"PING":6,"GOAWAY":7,"WINDOW_UPDATE":8,"CONTINUATION":9}

def http2_frame(frame_type_name, stream_id, payload, flags=0x04):
    """HTTP/2 frame header (9 bytes) + payload."""
    ftype = _HTTP2_TYPES.get(frame_type_name, 0)
    length= len(payload)
    hdr   = struct.pack(">I", length)[1:]                   
    hdr  += struct.pack(">BBi", ftype, flags, stream_id & 0x7FFFFFFF)
    return hdr + payload

def sba_http2_request(service, operation, body=b""):
    """
    Simplified HTTP/2 HEADERS + DATA frames for 5G SBA NF service operations.
    e.g. POST /namf-comm/v1/ue-contexts/{id}/n1-n2-messages
    """
                                                                   
    path    = f"/{service}/v1/{operation}".encode()
    headers = b"\x82\x84\x41" + bytes([len(path)]) + path                                      
    h2_hdr  = http2_frame("HEADERS", stream_id=random.randint(1,999)*2+1, payload=headers)
    h2_data = http2_frame("DATA",    stream_id=random.randint(1,999)*2+1, payload=body)
    return h2_hdr + h2_data

                                                                
_COAP_METHODS = {"GET":1,"POST":2,"PUT":3,"DELETE":4}
_COAP_CODES   = {"2.01":0x41,"2.04":0x44,"4.04":0x84,"5.00":0xA0}

def coap_pdu(method_or_code, token=None, path="/sensor/data", payload=b""):
    """CoAP message — Type=CON (0), version=1."""
    if token is None:
        token = random.randint(0,0xFFFFFFFF).to_bytes(4,"big")
    token_len = len(token)
    if isinstance(method_or_code, str) and "." in method_or_code:
        code = _COAP_CODES.get(method_or_code, 0x44)
    else:
        code = (_COAP_METHODS.get(method_or_code,2) << 5) | 0x01             
    msg_id = random.randint(0, 0xFFFF)
                                       
    first  = (1<<6) | (0<<4) | token_len                     
    hdr    = struct.pack(">BBH", first, code, msg_id) + token
                                                              
    opts = b""
    for seg in path.strip("/").split("/"):
        seg_b = seg.encode()
        opts += struct.pack("B", (11<<4)|len(seg_b)) + seg_b
                    
    return hdr + opts + (b"\xFF" + payload if payload else b"")

                                                                
def icmp_echo(src_ip, dst_ip, seq=1, payload_len=32):
    data = bytes([i & 0xFF for i in range(payload_len)])
    icmp_id  = random.randint(0,0xFFFF)
    icmp_hdr = struct.pack(">BBHHH", 8, 0, 0, icmp_id, seq)                   
    words    = struct.unpack(">%dH" % (len(icmp_hdr)//2), icmp_hdr)
    csum     = sum(words); csum = (csum>>16)+(csum&0xFFFF)
    csum     = ~(csum+(csum>>16)) & 0xFFFF
    icmp_pkt = struct.pack(">BBHHH", 8, 0, csum, icmp_id, seq) + data
    return ipv4_pkt(src_ip, dst_ip, 1, icmp_pkt)                

def arp_request(src_ip, dst_ip):
                                                                                 
    src_mac_b = bytes([0x02,0x00,0x00,0x00,(sum(bytes(src_ip,"ascii"))>>8)&0xFF,
                       sum(bytes(src_ip,"ascii"))&0xFF])
    spa = ipaddress.IPv4Address(src_ip).packed
    tpa = ipaddress.IPv4Address(dst_ip).packed
    arp = struct.pack(">HHBBH", 1, 0x0800, 6, 4, 1)
    arp += src_mac_b + spa + (b"\x00"*6) + tpa
                                                           
    return eth_frame("ff:ff:ff:ff:ff:ff", src_ip[:17].replace(".",":")[:17],
                     0x0806, arp) if False else\
           b"\xff\xff\xff\xff\xff\xff" + src_mac_b + struct.pack(">H",0x0806) + arp

                                                                
                                                           
                                                                

GNB_IP  = "192.168.1.1"                        
AMF_IP  = "192.168.10.10"         
UPF_IP  = "192.168.20.20"            
CU_IP   = "192.168.30.30"                
DU_IP   = "192.168.31.31"             
SBA_IP  = "192.168.50.50"                       
IOT_IP  = "10.200.0.1"                            

def _ue_ip(ue_id):
    ip_int = 0x0A000000 | (ue_id & 0xFFFF)
    return f"{(ip_int>>24)&0xFF}.{(ip_int>>16)&0xFF}.{(ip_int>>8)&0xFF}.{ip_int&0xFF}"

def _teid(ue_id, gnb_id):
    return ((gnb_id & 0xFF) << 16) | (ue_id & 0xFFFF)

def pcap_nas_registration(ue, gnb):
    """Registration Request → NAS + NGAP uplink transport."""
    ue_ip = _ue_ip(ue.id)
                                          
    rrc_req = rrc_pdu("RrcSetupRequest", ue.id, gnb.id,
                      struct.pack(">HH", ue.id, random.randint(0,0xFFFF)))
    rlc_rrc = rlc_am_pdu(ue.id % 65536, rrc_req)
    pdcp_rrc= pdcp_pdu(0, ue.id % 262144, rlc_rrc)
    mac_rrc = mac_pdu(0, pdcp_rrc, with_bsr=True)
    phy_frm = phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1,
                            ue.cqi or 5, ue.layers or 1, len(mac_rrc), "UL")
    pcap_engine.write("uu_radio", phy_frm + mac_rrc)

                                      
    nas_reg = nas_pdu("RegistrationRequest", ue.id,
                      struct.pack(">HBB", ue.id, 0x01, 0x00))                        
    rrc_ul  = rrc_pdu("UlInformationTransfer", ue.id, gnb.id, nas_reg)
    rlc_ul  = rlc_am_pdu((ue.id+1) % 65536, rrc_ul)
    pdcp_ul = pdcp_pdu(1, (ue.id+1) % 262144, rlc_ul)
    mac_ul  = mac_pdu(1, pdcp_ul)
    phy_ul  = phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1, ue.cqi or 5,
                            ue.layers or 1, len(mac_ul), "UL")
    pcap_engine.write("uu_radio", phy_ul + mac_ul)

                                                
    ngap_ie = ngap_pdu("InitialUEMessage", ue_id=ue.id, gnb_id=gnb.id,
                       extra=nas_reg)
    ngap_ip = ipv4_pkt(GNB_IP, AMF_IP, 132,
                       sctp_pkt(38412, 38412, _teid(ue.id, gnb.id)&0x7FFFFFFF,
                                _sctp_chunk(0, 0, ngap_ie)))
    pcap_engine.write("n2_ngap",
                      eth_frame(_gnb_mac(gnb.id), "02:00:ff:ff:ff:ff",
                                0x0800, ngap_ip))

def pcap_nas_auth(ue, gnb):
    """Authentication Request + Response."""
    rand_b = struct.pack(">Q", ue.rand_challenge)
    autn_b = struct.pack(">HQ", ue.usim_sqn & 0xFFFF, ue.rand_challenge ^ ue.usim_sqn)
                      
    nas_auth_req = nas_pdu("AuthenticationRequest", ue.id, rand_b + autn_b)
    rrc_dl = rrc_pdu("DlInformationTransfer", ue.id, gnb.id, nas_auth_req)
    pdcp_dl= pdcp_pdu(0, ue.id % 262144, rlc_am_pdu(ue.id%65536, rrc_dl))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1,
                                    ue.cqi or 5, 1, len(pdcp_dl), "DL") + mac_pdu(1, pdcp_dl))
                              
    res_star = hashlib.sha256(rand_b + autn_b).digest()[:16]
    nas_auth_resp = nas_pdu("AuthenticationResponse", ue.id, res_star)
    rrc_ul = rrc_pdu("UlInformationTransfer", ue.id, gnb.id, nas_auth_resp)
    pdcp_ul= pdcp_pdu(1, (ue.id+2)%262144, rlc_am_pdu((ue.id+1)%65536, rrc_ul))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1,
                                    ue.cqi or 5, 1, len(pdcp_ul), "UL") + mac_pdu(1, pdcp_ul))
                             
    ngap_ip = ipv4_pkt(GNB_IP, AMF_IP, 132,
                       sctp_pkt(38412, 38412, _teid(ue.id, gnb.id)&0x7FFFFFFF,
                                _sctp_chunk(0,0, ngap_pdu("UplinkNASTransport",
                                            ue_id=ue.id, gnb_id=gnb.id,
                                            extra=nas_auth_resp))))
    pcap_engine.write("n2_ngap",
                      eth_frame(_gnb_mac(gnb.id), "02:00:ff:ff:ff:ff",
                                0x0800, ngap_ip))
                                         
    body = json.dumps({"supi": f"imsi-00101{ue.id:010d}",
                       "rand": rand_b.hex(), "res": res_star.hex()}).encode()
    sba_pkt = sba_http2_request("nudm-ueau", f"authenticate/{ue.id}", body)
    sba_ip  = ipv4_pkt(AMF_IP, SBA_IP, 6,
                       udp_seg(443, 443, sba_pkt))
    pcap_engine.write("sba_http2",
                      eth_frame("02:00:ff:10:00:01","02:00:ff:50:00:01",0x0800,sba_ip))

def pcap_rrc_setup(ue, gnb):
    """RRC Setup + Security Mode + Reconfiguration."""
                           
    rrc_setup = rrc_pdu("RrcSetup", ue.id, gnb.id,
                        struct.pack(">HHHBB",
                                    ue.alloc_rb or 1, gnb.num_rb,
                                    int(gnb.freq_ghz*100) & 0xFFFF,
                                    gnb.tx_ant & 0xFF,
                                    gnb.max_layers & 0xFF))
    pdcp_dl = pdcp_pdu(0, ue.id%262144, rlc_am_pdu(ue.id%65536, rrc_setup))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1, ue.cqi or 5,
                                    ue.layers or 1, len(pdcp_dl), "DL")
                      + mac_pdu(1, pdcp_dl))
                             
    rrc_complete = rrc_pdu("RrcSetupComplete", ue.id, gnb.id)
    pdcp_ul = pdcp_pdu(1, (ue.id+10)%262144,
                       rlc_am_pdu((ue.id+5)%65536, rrc_complete))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1, ue.cqi or 5,
                                    ue.layers or 1, len(pdcp_ul), "UL")
                      + mac_pdu(1, pdcp_ul))
                                
    sec_cmd = rrc_pdu("SecurityModeCommand", ue.id, gnb.id,
                      struct.pack(">BB", 0x01, 0x01))                  
    pdcp_sc = pdcp_pdu(0, (ue.id+20)%262144, rlc_am_pdu((ue.id+10)%65536, sec_cmd))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1, ue.cqi or 5,
                                    ue.layers or 1, len(pdcp_sc), "DL")
                      + mac_pdu(1, pdcp_sc))
                                 
    sec_ok = rrc_pdu("SecurityModeComplete", ue.id, gnb.id)
    pdcp_so = pdcp_pdu(1, (ue.id+30)%262144, rlc_am_pdu((ue.id+15)%65536, sec_ok))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1, ue.cqi or 5,
                                    ue.layers or 1, len(pdcp_so), "UL")
                      + mac_pdu(1, pdcp_so))
                                           
    f1_pdu = f1ap_pdu("DLRRCMessageTransfer", du_id=gnb.id, cu_id=0,
                      ue_id=ue.id, payload=rrc_setup)
    f1_udp = udp_seg(38472, 38472, f1_pdu)
    f1_ip  = ipv4_pkt(CU_IP, DU_IP, 17, f1_udp)
    pcap_engine.write("f1_du_cu",
                      eth_frame("02:00:ff:30:00:01","02:00:ff:31:00:01",0x0800,f1_ip))

def pcap_pdu_session(ue, gnb):
    """PDU Session Establishment — NAS + NGAP + GTP-U tunnel setup."""
    ue_ip = _ue_ip(ue.id)
                                           
    nas_pdu_req = nas_pdu("PDUSessionEstReq", ue.id,
                          struct.pack(">HBB", 1, 0x01, 0x01))                 
    rrc_ul = rrc_pdu("UlInformationTransfer", ue.id, gnb.id, nas_pdu_req)
    pdcp_ul= pdcp_pdu(1, (ue.id+50)%262144, rlc_am_pdu((ue.id+25)%65536, rrc_ul))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, gnb.id, ue.alloc_rb or 1, ue.cqi or 5,
                                    ue.layers or 1, len(pdcp_ul), "UL")
                      + mac_pdu(4, pdcp_ul))
                                         
    ngap_sess = ngap_pdu("PDUSessionSetupReq", ue_id=ue.id, gnb_id=gnb.id,
                         extra=struct.pack(">4s", ipaddress.IPv4Address(ue_ip).packed))
    sctp_data = sctp_pkt(38412, 38412, _teid(ue.id, gnb.id)&0x7FFFFFFF,
                         _sctp_chunk(0,0, ngap_sess))
    ngap_ip = ipv4_pkt(GNB_IP, AMF_IP, 132, sctp_data)
    pcap_engine.write("n2_ngap",
                      eth_frame(_gnb_mac(gnb.id),"02:00:ff:ff:ff:ff",0x0800,ngap_ip))
                                     
    ngap_resp = ngap_pdu("PDUSessionSetupResp", ue_id=ue.id, gnb_id=gnb.id,
                         extra=struct.pack(">IH", _teid(ue.id, gnb.id), 1))
    ngap_resp_ip = ipv4_pkt(AMF_IP, GNB_IP, 132,
                            sctp_pkt(38412, 38412, _teid(ue.id,gnb.id)&0x7FFFFFFF,
                                     _sctp_chunk(0,0, ngap_resp)))
    pcap_engine.write("n2_ngap",
                      eth_frame("02:00:ff:ff:ff:ff",_gnb_mac(gnb.id),0x0800,ngap_resp_ip))
                                          
    inner_ip = ipv4_pkt(ue_ip, "8.8.8.8", 1, icmp_echo(ue_ip, "8.8.8.8", seq=1))
    gtp_pkt  = gtpu_pkt(_teid(ue.id, gnb.id), inner_ip)
    gtp_udp  = udp_seg(2152, 2152, gtp_pkt)
    gtp_ip   = ipv4_pkt(GNB_IP, UPF_IP, 17, gtp_udp)
    pcap_engine.write("n3_gtpu",
                      eth_frame(_gnb_mac(gnb.id),"02:00:ff:20:00:01",0x0800,gtp_ip))
                     
    pcap_engine.write("icmp_ctrl", arp_request(ue_ip, GNB_IP))
                                
    body = json.dumps({"supi":f"imsi-001-01-{ue.id:010d}",
                       "ueIp": ue_ip,
                       "teid": _teid(ue.id, gnb.id),
                       "slice": ue.slice}).encode()
    sba_pkt = sba_http2_request("nsmf-pdusession", f"sm-contexts/{ue.id}", body)
    sba_ip  = ipv4_pkt(AMF_IP, SBA_IP, 6, udp_seg(443,443, sba_pkt))
    pcap_engine.write("sba_http2",
                      eth_frame("02:00:ff:10:00:01","02:00:ff:50:00:01",0x0800,sba_ip))

def pcap_data_plane(ue, gnb, tick):
    """
    Per-tick user-plane traffic: PHY DCI + PDSCH/PUSCH + GTP-U + application.
    Called every simulation tick for UEs in UP state.
    """
    if not ue.pdu_active or ue.tput <= 0 or ue.in_ho:
        return
    ue_ip = ue.pdu_ip or _ue_ip(ue.id)
    tbs   = max(1, min(1400, int(ue.tput * 1e6 / 8 / 100)))              
    mcs   = max(0, min(28, ue.cqi * 2))

                                                     
    phy_dl = phy_dci_frame(ue.id, gnb.id, ue.alloc_rb, mcs,
                           ue.layers, tbs, "DL")
                         
    phy_ul = phy_dci_frame(ue.id, gnb.id, max(1, ue.alloc_rb//4), mcs,
                           1, tbs//4, "UL")                          
    dl_payload = bytes([random.randint(0,255) for _ in range(min(tbs,64))])
    ul_payload = bytes([random.randint(0,255) for _ in range(min(tbs//4,32))])

                                  
    lcid = {"EMBB":4,"URLLC":5,"MMTC":6}.get(ue.slice, 4)
    sn   = (tick * ue.id) & 0x3FFFF
    pdcp_dl_pdu = pdcp_pdu(lcid, sn, rlc_am_pdu(sn, dl_payload, poll=(tick%32==0)))
    pdcp_ul_pdu = pdcp_pdu(lcid, sn+1, rlc_am_pdu(sn+1, ul_payload))
    mac_dl  = mac_pdu(lcid, pdcp_dl_pdu)
    mac_ul  = mac_pdu(lcid, pdcp_ul_pdu, with_bsr=(tick%8==0))

    pcap_engine.write("uu_radio", phy_dl + mac_dl)
    pcap_engine.write("uu_radio", phy_ul + mac_ul)

                              
    if ue.slice == "EMBB":
                                      
        app_body = json.dumps({"ue":ue.id,"tick":tick,"sinr":round(ue.sinr,2),
                               "tput":round(ue.tput,2)}).encode()
        app_data = b"HTTP/2 DATA " + app_body[:48]
        inner_ip = ipv4_pkt(ue_ip, "1.1.1.1", 6,
                            udp_seg(random.randint(1024,65535), 443, app_data))
    elif ue.slice == "URLLC":
                                                                
        inner_ip = ipv4_pkt(ue_ip, "10.100.0.1", 17,
                            udp_seg(5001, 5001,
                                    struct.pack(">IHHH", tick, ue.id,
                                                int(ue.latency*100)&0xFFFF, 0)))
    else:        
                                                        
        coap_data = coap_pdu("POST", path="/sensor/telemetry",
                             payload=struct.pack(">IHH", tick, ue.id,
                                                 int(ue.sinr*10+200)&0xFFFF))
        inner_ip = ipv4_pkt(ue_ip, IOT_IP, 17, udp_seg(5683, 5683, coap_data))
                                      
        pcap_engine.write("coap_mmtc",
                          eth_frame(_ue_mac(ue.id),"02:00:ff:c8:00:01",
                                    0x0800,
                                    ipv4_pkt(ue_ip, IOT_IP, 17,
                                             udp_seg(5683,5683,coap_data))))

    gtp  = gtpu_pkt(_teid(ue.id, gnb.id), inner_ip)
    gtp_udp = udp_seg(2152, 2152, gtp)
    gtp_ip  = ipv4_pkt(GNB_IP, UPF_IP, 17, gtp_udp)
    pcap_engine.write("n3_gtpu",
                      eth_frame(_gnb_mac(gnb.id),"02:00:ff:20:00:01",
                                0x0800, gtp_ip))

                                   
    if tick % 10 == (ue.id % 10):
        icmp_pkt = icmp_echo(ue_ip, GNB_IP, seq=tick & 0xFFFF)
        pcap_engine.write("icmp_ctrl",
                          eth_frame(_ue_mac(ue.id), _gnb_mac(gnb.id),
                                    0x0800, icmp_pkt))

def pcap_handover(ue, src_gnb, dst_gnb):
    """Xn/N2-based handover: MeasurementReport → HO Required → HO Request → HO Complete."""
                                           
    meas = rrc_pdu("MeasurementReport", ue.id, src_gnb.id,
                   struct.pack(">HhH", dst_gnb.id,
                               int(ue.sinr*10), int(ue.dist_to_gnb)))
    pdcp_meas = pdcp_pdu(0, ue.id%262144, rlc_am_pdu(ue.id%65536, meas))
    pcap_engine.write("uu_radio",
                      phy_dci_frame(ue.id, src_gnb.id, ue.alloc_rb or 1,
                                    ue.cqi or 3, 1, len(pdcp_meas), "UL")
                      + mac_pdu(1, pdcp_meas))
                                         
    ho_req = ngap_pdu("HandoverRequired", ue_id=ue.id, gnb_id=src_gnb.id,
                      extra=struct.pack(">HH", dst_gnb.id, int(ue.dist_to_gnb)))
    ngap_ip = ipv4_pkt(GNB_IP, AMF_IP, 132,
                       sctp_pkt(38412,38412,_teid(ue.id,src_gnb.id)&0x7FFFFFFF,
                                _sctp_chunk(0,0, ho_req)))
    pcap_engine.write("n2_ngap",
                      eth_frame(_gnb_mac(src_gnb.id),"02:00:ff:ff:ff:ff",
                                0x0800, ngap_ip))
                                        
    ho_rq  = ngap_pdu("HandoverRequest", ue_id=ue.id, gnb_id=dst_gnb.id)
    ngap_ip2 = ipv4_pkt(AMF_IP, GNB_IP, 132,
                        sctp_pkt(38412,38412,_teid(ue.id,dst_gnb.id)&0x7FFFFFFF,
                                 _sctp_chunk(0,0, ho_rq)))
    pcap_engine.write("n2_ngap",
                      eth_frame("02:00:ff:ff:ff:ff",_gnb_mac(dst_gnb.id),
                                0x0800, ngap_ip2))
                                         
    body = json.dumps({"ueId":ue.id,"srcGnb":src_gnb.id,"dstGnb":dst_gnb.id}).encode()
    sba_pkt = sba_http2_request("nsmf-pdusession",f"sm-contexts/{ue.id}/modify",body)
    sba_ip  = ipv4_pkt(AMF_IP, SBA_IP, 6, udp_seg(443,443,sba_pkt))
    pcap_engine.write("sba_http2",
                      eth_frame("02:00:ff:10:00:01","02:00:ff:50:00:01",
                                0x0800, sba_ip))

def pcap_ric_e2(gnb, ue_list, tick):
    """Near-RT RIC E2 indication report: F1AP telemetry DU→CU."""
    payload = json.dumps({
        "gnbId": gnb.id, "tick": tick,
        "avgSinr": round(gnb.e2_avg_sinr,2),
        "avgBler": round(gnb.e2_avg_bler,4),
        "load":    round(gnb.e2_load,3),
        "ues":     len(ue_list),
    }).encode()
    f1_pdu_data = f1ap_pdu("ULRRCMessageTransfer", du_id=gnb.id, cu_id=0,
                           ue_id=0, payload=payload)
    f1_udp = udp_seg(38472, 38472, f1_pdu_data)
    f1_ip  = ipv4_pkt(DU_IP, CU_IP, 17, f1_udp)
    pcap_engine.write("f1_du_cu",
                      eth_frame("02:00:ff:31:00:01","02:00:ff:30:00:01",
                                0x0800, f1_ip))

def pcap_sba_nwdaf(analytics):
    """NWDAF analytics subscription notification over SBA HTTP/2."""
    body = json.dumps(analytics).encode()
    pkt  = sba_http2_request("nnwdaf-analyticsinfo","subscriptions/notify", body)
    ip   = ipv4_pkt(SBA_IP, AMF_IP, 6, udp_seg(443,443,pkt))
    pcap_engine.write("sba_http2",
                      eth_frame("02:00:ff:50:00:01","02:00:ff:10:00:01",
                                0x0800, ip))

                                                                             
                                                                 
 
                                                                           
                                                             
                                                                             

                        
NS3_MS_PER_SLOT_15KHZ  = 1.0
NS3_MS_PER_SLOT_30KHZ  = 0.5
NS3_MS_PER_SLOT_60KHZ  = 0.25
NS3_MS_PER_SLOT_120KHZ = 0.125
NS3_SYMBOLS_PER_SLOT   = 14
NS3_SC_PER_RB          = 12
NS3_OFDM_CP_OVERHEAD   = 144.0/2048
NS3_SPEED_OF_LIGHT     = 2.998e8

class NsLteAmc:
    """
    ns3::LteAmc — Adaptive Modulation and Coding.

    When HAS_NS3_LTE is True the three hot-path static methods delegate to
    the native C++ _Ns3LteAmc object (ns3.lte module) for exact 3GPP table
    lookups at C++ speed.  The pure-Python tables remain as the fallback and
    are also used by GetBler() which has no direct NS-3 equivalent.
    """
    SPECTRAL_EFF_FOR_CQI = [
        0.0, 0.1523, 0.2344, 0.3770, 0.6016, 0.8770, 1.1758,
        1.4766, 1.9141, 2.4063, 2.7305, 3.3223, 3.9023, 4.5234, 5.1152, 5.5547,
    ]
    MOD_ORDER_FOR_CQI = [0, 2, 2, 2, 2, 4, 4, 6, 6, 6, 6, 6, 6, 8, 8, 8]
    CODE_RATE_FOR_CQI = [
        (0, 1024), (78, 1024), (120, 1024), (193, 1024), (308, 1024),
        (449, 1024), (602, 1024), (378, 1024), (490, 1024), (616, 1024),
        (466, 512), (567, 512), (666, 512), (772, 1024), (873, 1024), (948, 1024),
    ]
    MCS_TO_ITBS = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
        9, 10, 11, 12, 13, 14, 15, 15, 16, 17,
        18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
    ]

    @staticmethod
    def GetCqiFromSpectralEfficiency(sinr_db: float) -> int:
        """ns3::LteAmc::GetCqiFromSpectralEfficiency() — maps SINR→CQI.
        NOTE: always uses the Python lookup table even when HAS_NS3_LTE is True.
        Dispatching per-UE per-tick calls through _ns3_call() saturates the
        main-thread queue (100-400 calls/tick × 1s tick = queue never drains).
        The Python table is the same 3GPP TS 36.213 Table 7.2.3-1 that NS-3 uses."""
        if sinr_db < -6.0: return 0
        sinr_lin = 10.0 ** (sinr_db / 10.0)
        spec_eff = math.log2(1.0 + sinr_lin)
        table = NsLteAmc.SPECTRAL_EFF_FOR_CQI
        cqi = 1
        for i in range(1, 16):
            if spec_eff >= table[i]: cqi = i
            else: break
        return max(1, min(15, cqi))

    @staticmethod
    def GetMcsFromCqi(cqi: int) -> int:
        # Python table — same as NS-3's CQI_TO_MCS; avoids _ns3_call queue saturation.
        CQI_TO_MCS = [0, 0, 1, 2, 4, 6, 8, 11, 13, 15, 18, 20, 22, 24, 26, 28]
        return CQI_TO_MCS[max(0, min(15, cqi))]

    @staticmethod
    def GetTbSizeFromMcsNprb(mcs: int, nprb: int) -> int:
        # Python table — same formula NS-3 uses internally.
        mcs = max(0, min(28, mcs)); nprb = max(1, min(110, nprb))
        itbs = NsLteAmc.MCS_TO_ITBS[mcs]
        se   = NsLteAmc.SPECTRAL_EFF_FOR_CQI[min(itbs, 15)]
        bits = nprb * se * 12.0 * 14.0
        return max(1, int(bits / 8.0))

    @staticmethod
    def GetBler(sinr_db: float, cqi: int, harq_round: int = 0) -> float:
        """
        BLER logistic curve calibrated to 3GPP link-level results.
        No direct NS-3 equivalent — always uses the Python model.

        Root cause of old bug: GetCqiFromSpectralEfficiency assigns the highest
        CQI whose Shannon capacity <= actual capacity, so SINR always sits right
        at the CQI minimum threshold.  Real outer-loop link adaptation adds a
        ~4 dB SINR margin above the minimum threshold before selecting that CQI,
        keeping operational BLER at 1-10%.  We model this with OLM=4.0 dB shift.

        Logistic slope=0.8 gives ~8 dB transition width (10% BLER at threshold,
        <1% at threshold+5 dB, >90% at threshold-5 dB).
        """
        if cqi <= 0: return 1.0
                                                                                     
        se           = NsLteAmc.SPECTRAL_EFF_FOR_CQI[cqi]
        sinr_lin_min = max(2.0 ** se - 1.0, 1e-6)
        sinr_thresh  = 10.0 * math.log10(sinr_lin_min) + 1.5
        OLM = 4.0                                         
        eff_sinr = sinr_db + 3.0 * harq_round
        margin   = eff_sinr - sinr_thresh + OLM
        return 1.0 / (1.0 + math.exp(0.8 * margin))

class NsThreeGppUmiStreetCanyonPropagationLossModel:
    """
    ns3::ThreeGppUmiStreetCanyonPropagationLossModel (TR 38.901 Table 7.4.1-1).

    When HAS_NS3_PROPAGATION is True, DoCalcRxPower instantiates the real
    C++ model and calls CalculateTxPower() as a pure math function — no
    packet simulation needed.  GetLossLos / GetLossNlos / GetPlos remain
    Python because NS-3 exposes them only as internal (protected) methods;
    they are used for reference and by the shadowing model.
    """

                                                                           
    _ns3_model_cache: dict = {}                             

    # Pool of pre-created mob node pairs for main-thread NS-3 calls.
    # Index into the pool is grabbed via a counter; since _ns3_drain runs
    # sequentially on the main thread there is never concurrent access.
    _ns3_mob_pool: list = []   # list of (mob_a, mob_b) pairs
    _MOB_POOL_SIZE = 1         # one pair is enough — drain is sequential

    @staticmethod
    def _ensure_mob_pool():
        """Create the mob node pool on the main thread (lazy, once)."""
        cls = NsThreeGppUmiStreetCanyonPropagationLossModel
        while len(cls._ns3_mob_pool) < cls._MOB_POOL_SIZE:
            a = _ns3_create_object("ConstantPositionMobilityModel")
            b = _ns3_create_object("ConstantPositionMobilityModel")
            cls._ns3_mob_pool.append((a, b))

    @staticmethod
    def _GetBreakpointDistance(fc_ghz, h_bs, h_ut):
        return 4.0 * (h_bs - 1.0) * (h_ut - 1.0) * (fc_ghz * 1e9) / NS3_SPEED_OF_LIGHT

    @staticmethod
    def GetLossLos(d2d, d3d, fc_ghz, h_bs=10.0, h_ut=1.5):
        d2d = max(d2d, 1.0); d3d = max(d3d, 1.0)
        d_bp = NsThreeGppUmiStreetCanyonPropagationLossModel._GetBreakpointDistance(fc_ghz, h_bs, h_ut)
        if d2d < d_bp:
            return 32.4 + 21.0 * math.log10(d3d) + 20.0 * math.log10(fc_ghz)
        return (32.4 + 40.0 * math.log10(d3d) + 20.0 * math.log10(fc_ghz)
                - 9.5 * math.log10(d_bp**2 + (h_bs - h_ut)**2))

    @staticmethod
    def GetLossNlos(d2d, d3d, fc_ghz, h_bs=10.0, h_ut=1.5):
        pl_los  = NsThreeGppUmiStreetCanyonPropagationLossModel.GetLossLos(d2d, d3d, fc_ghz, h_bs, h_ut)
        pl_nlos = (35.3 * math.log10(d3d) + 22.4 + 21.3 * math.log10(fc_ghz) - 0.3 * (h_ut - 1.5))
        return max(pl_los, pl_nlos)

    @staticmethod
    def GetPlos(d2d):
        d = max(d2d, 1.0)
        return min(18.0 / d, 1.0) * (1.0 - math.exp(-d / 36.0)) + math.exp(-d / 36.0)

    @staticmethod
    def DoCalcRxPower(tx_dbm, d2d, d3d, fc_ghz, h_bs=10.0, h_ut=1.5):
        # DESIGN DECISION: always use the pure-Python TR 38.901 path.
        #
        # Rationale: the NS-3 C++ path computes exactly the same formula
        # (ThreeGppUmiStreetCanyonPropagationLossModel IS the TR 38.901 math).
        # Dispatching it through _ns3_call() adds up to server.timeout (500 ms)
        # of latency per UE per tick because the main thread is blocked in
        # select() between handle_request() calls.  With 100-400 UEs each
        # requiring ~6 path-loss evaluations per tick, the queue grows without
        # bound and eventually the heap is corrupted by the pile-up of blocked
        # background threads and their closure captures.
        #
        # The Python path below is used unchanged; NS-3 objects are still
        # initialised (for LteAmc table lookups which are called rarely and
        # tolerate the queue latency).
        p_los   = NsThreeGppUmiStreetCanyonPropagationLossModel.GetPlos(d2d)
        pl_los  = NsThreeGppUmiStreetCanyonPropagationLossModel.GetLossLos(d2d, d3d, fc_ghz, h_bs, h_ut)
        pl_nlos = NsThreeGppUmiStreetCanyonPropagationLossModel.GetLossNlos(d2d, d3d, fc_ghz, h_bs, h_ut)
        return tx_dbm - (p_los * pl_los + (1.0 - p_los) * pl_nlos)

class NsThreeGppShadowingModel:
    """TR 38.901 UMi shadow fading: AR(1) log-normal."""
    SIGMA_LOS_DB  = 4.0
    SIGMA_NLOS_DB = 7.82
    D_CORR_M      = 10.0

    @staticmethod
    def UpdateShadow(prev_db, delta_dist_m, is_los):
        sigma = NsThreeGppShadowingModel.SIGMA_LOS_DB if is_los else NsThreeGppShadowingModel.SIGMA_NLOS_DB
        alpha = math.exp(-delta_dist_m / NsThreeGppShadowingModel.D_CORR_M)
        innov = random.gauss(0.0, sigma) * math.sqrt(max(0.0, 1.0 - alpha * alpha))
        return alpha * prev_db + innov

class NsThreeGppChannelModel:
    """ns3::ThreeGppChannelModel fast fading (Rician/Rayleigh)."""
    K_FACTOR_LOS_DB = 9.0

    @staticmethod
    def GetFastFadingDb(is_los, v_ms, fc_ghz):
        if is_los:
            K_lin = 10.0 ** (NsThreeGppChannelModel.K_FACTOR_LOS_DB / 10.0)
            nu    = math.sqrt(K_lin / (K_lin + 1.0))
            sigma = math.sqrt(1.0 / (2.0 * (K_lin + 1.0)))
            re = random.gauss(nu, sigma); im = random.gauss(0.0, sigma)
        else:
            re = random.gauss(0.0, 1.0 / math.sqrt(2.0))
            im = random.gauss(0.0, 1.0 / math.sqrt(2.0))
        amp = math.sqrt(re * re + im * im)
        return 20.0 * math.log10(max(amp, 1e-9))

    @staticmethod
    def GetDopplerHz(v_ms, fc_ghz):
        return v_ms * (fc_ghz * 1e9) / NS3_SPEED_OF_LIGHT

class NsLteInterference:
    """
    ns3::LteInterference — SINR component computation.

    GetMassiveMimoBeamformingGainDb and GetMrcGainDb delegate to the native
    ns3.antenna / ns3.spectrum MatrixBasedChannelModel when HAS_NS3_ANTENNA
    is True, giving exact 3GPP spatial-geometry results.  All other methods
    remain in Python (they are already exact TR 38.901 formulas).
    """
    BOLTZMANN_K = 1.380649e-23
    TEMP_KELVIN = 290.0

    @staticmethod
    def GetNoisePowerDbm(bw_hz, noise_figure_db):
        N_watts = NsLteInterference.BOLTZMANN_K * NsLteInterference.TEMP_KELVIN * bw_hz
        return 10.0 * math.log10(max(N_watts, 1e-30)) + 30.0 + noise_figure_db

    @staticmethod
    def GetMrcGainDb(n_rx_ant):
        if HAS_NS3_ANTENNA:
                                                                         
                                                                                  
            try:
                upa = _Ns3UPA()
                upa.SetAttribute("NumColumns", _Ns3DoubleValue(n_rx_ant))
                upa.SetAttribute("NumRows",    _Ns3DoubleValue(1))
                return upa.GetGainDb(0.0, 0.0)                                       
            except Exception:
                pass                          
        return 10.0 * math.log10(max(1, n_rx_ant))

    @staticmethod
    def GetMassiveMimoBeamformingGainDb(n_tx_ant, n_layers):
        if HAS_NS3_ANTENNA:
                                                                                        
                                                                              
            try:
                upa = _Ns3UPA()
                upa.SetAttribute("NumColumns", _Ns3DoubleValue(n_tx_ant))
                upa.SetAttribute("NumRows",    _Ns3DoubleValue(1))
                array_gain_db = upa.GetGainDb(0.0, 0.0)
                spatial_mux_penalty_db = 3.0 * (n_layers - 1)
                return array_gain_db - spatial_mux_penalty_db
            except Exception:
                pass                          
        return 10.0 * math.log10(max(1, n_tx_ant)) - 3.0 * (n_layers - 1)

    @staticmethod
    def GetInterCellInterferenceDbm(ue_x, ue_y, interferer_positions, tx_dbm, fc_ghz, h_bs=10.0, h_ut=1.5):
        total_mw = 0.0
        for (nx, ny) in interferer_positions:
            d2d = max(1.0, math.sqrt((ue_x - nx)**2 + (ue_y - ny)**2))
            d3d = math.sqrt(d2d**2 + (h_bs - h_ut)**2)
            rx_dbm = NsThreeGppUmiStreetCanyonPropagationLossModel.DoCalcRxPower(tx_dbm, d2d, d3d, fc_ghz, h_bs, h_ut)
            rx_dbm += random.gauss(0.0, 4.0)
            total_mw += 10.0 ** (rx_dbm / 10.0)
        if total_mw < 1e-20: return -200.0
        return 10.0 * math.log10(total_mw)

    @staticmethod
    def GetIntraCellInterferenceDbm(n_active_ues, n_tx_ant, bw_mhz, noise_figure_db):
        bw_hz     = bw_mhz * 1e6
        noise_dbm = NsLteInterference.GetNoisePowerDbm(bw_hz, noise_figure_db)
        spatial_cap = n_tx_ant / 2.0
        leak_db = -25.0 if n_active_ues <= max(1, spatial_cap) else\
                  -25.0 + 10.0 * math.log10(n_active_ues / spatial_cap)
        ul_overload = max(0, n_active_ues - 270)
        noise_rise  = 0.3 * ul_overload / max(1, 270)
        return noise_dbm + leak_db + noise_rise

class NsLteSpectrumPhy:
    """ns3::LteSpectrumPhy::ComputeSinr() full pipeline."""
    NOISE_FIGURE_DB = 7.0

    @staticmethod
    def ComputeSinr(d2d, ue_x, ue_y, shadow_db, fast_db, n_layers, tx_dbm, fc_ghz,
                    bw_mhz, n_tx_ant, n_rx_ant, h_bs, interferer_positions, n_active_ues=1):
        d3d     = math.sqrt(d2d**2 + (h_bs - 1.5)**2)
        rx_dbm  = NsThreeGppUmiStreetCanyonPropagationLossModel.DoCalcRxPower(tx_dbm, d2d, d3d, fc_ghz, h_bs)
        rx_dbm += shadow_db + fast_db
        rx_dbm += NsLteInterference.GetMassiveMimoBeamformingGainDb(n_tx_ant, n_layers)
        rx_dbm += NsLteInterference.GetMrcGainDb(n_rx_ant)
        noise_dbm = NsLteInterference.GetNoisePowerDbm(bw_mhz * 1e6, NsLteSpectrumPhy.NOISE_FIGURE_DB)
        ici_dbm   = NsLteInterference.GetInterCellInterferenceDbm(
                        ue_x, ue_y, interferer_positions, tx_dbm, fc_ghz, h_bs)
        intra_dbm = NsLteInterference.GetIntraCellInterferenceDbm(
                        n_active_ues, n_tx_ant, bw_mhz, NsLteSpectrumPhy.NOISE_FIGURE_DB)
        S_mw     = 10.0 ** (rx_dbm / 10.0)
        N_mw     = 10.0 ** (noise_dbm / 10.0)
        Iici_mw  = 10.0 ** (ici_dbm / 10.0)
        Iintra_mw= 10.0 ** (intra_dbm / 10.0)
        sinr_lin = S_mw / max(N_mw + Iici_mw + Iintra_mw, 1e-20)
        sinr_db  = 10.0 * math.log10(max(sinr_lin, 1e-9))
        return max(-15.0, min(35.0, sinr_db))

class NsNrPhy:
    """ns3::NrPhy helpers."""
    @staticmethod
    def ComputePdcchDecodeProb(sinr_db):
        return 1.0 / (1.0 + math.exp(-1.5 * (sinr_db + 2.0)))

    @staticmethod
    def ComputeRsrp(tx_dbm, pl_db, shadow_db, fast_db):
        return tx_dbm - pl_db - shadow_db + fast_db

    @staticmethod
    def ComputeRsrq(rsrp_dbm, noise_dbm):
        tot_mw = 10.0**(rsrp_dbm/10.0) + 10.0**(noise_dbm/10.0)
        return rsrp_dbm - 10.0 * math.log10(max(tot_mw, 1e-20))

class NsNrMacScheduler:
    """ns3::NrMacSchedulerNs3 — NR numerology and resource grid."""
    NUMEROLOGY_TO_SCS = {0: 15000, 1: 30000, 2: 60000, 3: 120000, 4: 240000}

    @staticmethod
    def GetScsHz(fc_ghz):
        if   fc_ghz <= 1.0:   return 15000
        elif fc_ghz <= 6.0:   return 30000
        elif fc_ghz <= 24.0:  return 60000
        elif fc_ghz <= 52.6:  return 120000
        else:                  return 240000

    @staticmethod
    def GetSlotsPerSecond(scs_hz):
        return 1000.0 * (scs_hz / 15000.0)

    @staticmethod
    def GetNumRbFromBwScs(bw_hz, scs_hz):
        return max(1, int(math.floor(bw_hz / (scs_hz * NS3_SC_PER_RB))))

    @staticmethod
    def GetOfdmEfficiency(scs_hz):
        base_scs = 15000.0; ratio = base_scs / scs_hz
        fft_size = 2048.0 * ratio; cp_size  = 144.0  * ratio
        return (fft_size / (fft_size + cp_size)) * 0.90

    @staticmethod
    def ComputeThroughputMbps(sinr_db, alloc_rb, n_layers, cqi, fc_ghz, bw_mhz, total_rb):
        if cqi <= 0 or alloc_rb == 0: return 0.0
        cr_num, cr_den = NsLteAmc.CODE_RATE_FOR_CQI[cqi]
        qm   = NsLteAmc.MOD_ORDER_FOR_CQI[cqi]; cr = cr_num / cr_den
        scs  = NsNrMacScheduler.GetScsHz(fc_ghz)
        sps  = NsNrMacScheduler.GetSlotsPerSecond(scs)
        oeff = NsNrMacScheduler.GetOfdmEfficiency(scs)
        bits_per_slot = qm * cr * NS3_SC_PER_RB * NS3_SYMBOLS_PER_SLOT * n_layers * alloc_rb
        bler     = NsLteAmc.GetBler(sinr_db, cqi)
        harq_eff = 1.0 - bler * 0.25
        pdcch_ok = NsNrPhy.ComputePdcchDecodeProb(sinr_db)
        return max(0.0, bits_per_slot * oeff * harq_eff * pdcch_ok * sps / 1e6)

class NsPfFfMacScheduler:
    """ns3::PfFfMacScheduler — Proportional Fair."""
    ALPHA = 0.1

    @staticmethod
    def UpdateAvgRate(avg_rate, instant_rate):
        return (1.0 - NsPfFfMacScheduler.ALPHA) * avg_rate + NsPfFfMacScheduler.ALPHA * instant_rate

    @staticmethod
    def GetMetric(instant_rate, avg_rate):
        return instant_rate / max(avg_rate, 1e-6)

    @staticmethod
    def Schedule(ues, rb_pool):
        if not ues: return
        ues.sort(key=lambda u: -u.pf_metric)
        rpu = rb_pool // len(ues); rem = rb_pool - rpu * len(ues)
        for i, u in enumerate(ues): u.alloc_rb = rpu + (rem if i == 0 else 0)

class NsRrFfMacScheduler:
    """ns3::RrFfMacScheduler — Round Robin."""
    @staticmethod
    def Schedule(ues, rb_pool):
        if not ues: return
        random.shuffle(ues)
        rpu = rb_pool // len(ues); rem = rb_pool - rpu * len(ues)
        for i, u in enumerate(ues): u.alloc_rb = rpu + (rem if i == 0 else 0)

class NsMaxCqiFfMacScheduler:
    """ns3::FdbetFfMacScheduler — Max-CQI greedy."""
    @staticmethod
    def Schedule(ues, rb_pool):
        if not ues: return
        ues.sort(key=lambda u: -u.cqi)
        rpu = rb_pool // len(ues); rem = rb_pool - rpu * len(ues)
        for i, u in enumerate(ues): u.alloc_rb = rpu + (rem if i == 0 else 0)

class NsLteUeRrc:
    """ns3::LteUeRrc state machine."""
    STATE_DEREG   = "DEREG"
    STATE_AUTH    = "AUTH?"
    STATE_AUTH_OK = "AUTH_OK"
    STATE_REG     = "REG"
    STATE_PDU     = "PDU"
    STATE_UP      = "UP"
    A3_OFFSET_DB  = 3.0
    A3_TTT_MS     = 256
    RSRP_HO_THRESHOLD_DBM = -110.0

    @staticmethod
    def ShouldTriggerHandover(rsrp_dbm):
        return rsrp_dbm < NsLteUeRrc.RSRP_HO_THRESHOLD_DBM

class NsLteLatencyModel:
    """ns3::FlowMonitor end-to-end latency (RLC+PDCP+MAC+queuing)."""
    UE_PROC_SLOTS  = 3
    GNB_PROC_SLOTS = 3
    N1_SLOTS       = 2
    N2_SLOTS       = 2

    @staticmethod
    def ComputeLatencyMs(slice_type, sinr_db, dist_m, n_active_ues, total_rbs, scs_hz):
        slot_ms  = 1000.0 / NsNrMacScheduler.GetSlotsPerSecond(scs_hz)
        n_tti    = 2 if slice_type == "URLLC" else 4
        tti_ms   = slot_ms * n_tti
        proc_ms  = (NsLteLatencyModel.UE_PROC_SLOTS + NsLteLatencyModel.GNB_PROC_SLOTS) * slot_ms
        prop_ms  = (dist_m / NS3_SPEED_OF_LIGHT) * 1000.0
        core_ms  = 0.3 if slice_type == "URLLC" else 1.0
        cqi_e    = NsLteAmc.GetCqiFromSpectralEfficiency(sinr_db)
        bler     = NsLteAmc.GetBler(sinr_db, cqi_e)
        rtt_ms   = 8.0 * slot_ms
        retx_ms  = bler * rtt_ms
        rho = min(0.95, n_active_ues / max(1, total_rbs))
        if slice_type == "URLLC":
            queue_ms = 0.2 * (1.0 + rho / max(1e-9, 1.0 - rho))
            return max(1.0, min(4.0, tti_ms + proc_ms + prop_ms + core_ms + retx_ms + queue_ms))
        elif slice_type == "MMTC":
            queue_ms = 15.0 * (1.0 + rho / (2.0 * max(1e-9, 1.0 - rho)))
            return max(5.0, tti_ms + proc_ms + prop_ms + core_ms + retx_ms + queue_ms)
        else:
            queue_ms = 4.0 * (1.0 + rho / (2.0 * max(1e-9, 1.0 - rho)))
            return max(2.0, min(50.0, tti_ms + proc_ms + prop_ms + core_ms + retx_ms + queue_ms))

                                                                             
                       
                                                                             
CFG = {
    "GNB_TX_DBM":     46.0,
    "GNB_HEIGHT_M":   10.0,
    "UE_HEIGHT_M":    1.5,
    "FREQ_GHZ":       28.0,
    "BW_MHZ":         400.0,
    "NOISE_FIG_DB":   7.0,
    "THERMAL_DBM_HZ": -174.0,
    "NUM_RB":         270,
    "RB_URLLC":       27,
    "RB_MMTC":        27,
    "RB_EMBB":        216,
    "SCHEDULER":      "PF",
    "GNB_TX_ANT":     64,
    "UE_RX_ANT":      4,
    "MAX_LAYERS":     4,
    "FFT_SIZE":       4096,
    "CP_SAMPLES":     288,
    "SCS_HZ":         30000.0,
    "CELL_RADIUS_M":  500.0,
    "ISD_M":          1000.0,
    "WEATHER":        "NORMAL",
    "TICK_MS":        1000,
}

POOL_SIZE_REAL  = 200
TARGET_MIN_REAL = 100
TARGET_MAX_REAL = 150
POOL_SIZE_TWIN  = 400
HTTP_PORT       = 9095

WEATHER_EFFECTS = {
    "NORMAL": {"pl_add": 0.0,  "shadow_scale": 1.0, "fading_scale": 1.0, "speed_scale": 1.0},
    "RAINY":  {"pl_add": 8.0,  "shadow_scale": 1.4, "fading_scale": 1.3, "speed_scale": 0.7},
    "WINDY":  {"pl_add": 2.0,  "shadow_scale": 1.1, "fading_scale": 1.1, "speed_scale": 1.6},
    "FOGGY":  {"pl_add": 12.0, "shadow_scale": 1.6, "fading_scale": 1.5, "speed_scale": 0.5},
}

def weather_fx():
    return WEATHER_EFFECTS.get(CFG["WEATHER"], WEATHER_EFFECTS["NORMAL"])

                                                     
CQI_TABLE = [
    (0, 0.000, 0.0000),
    (2, 0.076, 0.1523), (2, 0.117, 0.2344), (2, 0.188, 0.3770),
    (2, 0.301, 0.6016), (4, 0.220, 0.8770), (4, 0.294, 1.1758),
    (4, 0.369, 1.4766), (6, 0.322, 1.9141), (6, 0.433, 2.4063),
    (6, 0.538, 2.7305), (6, 0.588, 3.3223), (6, 0.650, 3.9023),
    (6, 0.754, 4.5234), (6, 0.853, 5.1152), (8, 0.926, 5.5547),
]
MOD_NAMES   = {2: "QPSK", 4: "16QAM", 6: "64QAM", 8: "256QAM"}
SLICE_NAMES = {"EMBB": "eMBB", "URLLC": "URLLC", "MMTC": "mMTC"}

                                                                            

def nr_scs_hz(freq_ghz):
    return NsNrMacScheduler.GetScsHz(freq_ghz)

def slots_per_second(scs_hz):
    return NsNrMacScheduler.GetSlotsPerSecond(scs_hz)

def ofdm_efficiency(scs_hz=None):
    return NsNrMacScheduler.GetOfdmEfficiency(scs_hz or int(CFG["SCS_HZ"]))

def num_rb_from_bw(bw_mhz, freq_ghz):
    scs = NsNrMacScheduler.GetScsHz(freq_ghz)
    return NsNrMacScheduler.GetNumRbFromBwScs(bw_mhz * 1e6, scs)

def umi_path_loss(dist_m, freq_ghz, gnb_h=None, ue_h=None):
    gnb_h = gnb_h if gnb_h is not None else CFG["GNB_HEIGHT_M"]
    ue_h  = ue_h  if ue_h  is not None else CFG["UE_HEIGHT_M"]
    d2d   = max(dist_m, 1.0)
    d3d   = math.sqrt(d2d**2 + (gnb_h - ue_h)**2)
    wfx   = weather_fx()
    p_los   = NsThreeGppUmiStreetCanyonPropagationLossModel.GetPlos(d2d)
    pl_los  = NsThreeGppUmiStreetCanyonPropagationLossModel.GetLossLos(d2d, d3d, freq_ghz, gnb_h, ue_h)
    pl_nlos = NsThreeGppUmiStreetCanyonPropagationLossModel.GetLossNlos(d2d, d3d, freq_ghz, gnb_h, ue_h)
    return p_los * pl_los + (1.0 - p_los) * pl_nlos + wfx["pl_add"]

def update_shadow(prev, speed):
    wfx        = weather_fx()
    delta_dist = speed * 1.0
    sigma      = NsThreeGppShadowingModel.SIGMA_LOS_DB * wfx["shadow_scale"]
    alpha      = math.exp(-delta_dist / NsThreeGppShadowingModel.D_CORR_M)
    innov      = random.gauss(0.0, sigma) * math.sqrt(max(0.0, 1.0 - alpha * alpha))
    return alpha * prev + innov

def fast_fading(speed, los):
    wfx = weather_fx()
    return NsThreeGppChannelModel.GetFastFadingDb(los, speed, CFG["FREQ_GHZ"]) * wfx["fading_scale"]

def doppler_hz(speed, freq_ghz=None):
    return NsThreeGppChannelModel.GetDopplerHz(speed, freq_ghz or CFG["FREQ_GHZ"])

def beamform_gain(tx_ant, layers):
    return NsLteInterference.GetMassiveMimoBeamformingGainDb(tx_ant, layers)

def inter_cell_interference_dbm(ux, uy, gnb_positions, tx_dbm, freq_ghz):
    return NsLteInterference.GetInterCellInterferenceDbm(
        ux, uy, gnb_positions, tx_dbm, freq_ghz, CFG["GNB_HEIGHT_M"], CFG["UE_HEIGHT_M"])

def intra_cell_interference_dbm(active_ues, bw_mhz, tx_ant):
    return NsLteInterference.GetIntraCellInterferenceDbm(
        active_ues, tx_ant, bw_mhz, CFG["NOISE_FIG_DB"])

def compute_sinr(dist_to_gnb, ux, uy, shadow_db, fast_db, layers,
                 tx_dbm, freq_ghz, bw_mhz, tx_ant, rx_ant, gnb_h,
                 interferer_positions, active_ues=1):
    return NsLteSpectrumPhy.ComputeSinr(
        dist_to_gnb, ux, uy, shadow_db, fast_db, layers,
        tx_dbm, freq_ghz, bw_mhz, tx_ant, rx_ant, gnb_h,
        interferer_positions, active_ues)

def sinr_to_cqi(sinr_db):
    return NsLteAmc.GetCqiFromSpectralEfficiency(sinr_db)

def ldpc_bler(sinr, cqi, harq_round=0):
    return NsLteAmc.GetBler(sinr, cqi, harq_round)

def polar_dci_prob(sinr):
    return NsNrPhy.ComputePdcchDecodeProb(sinr)

def compute_tput(sinr, alloc_rb, layers, cqi, freq_ghz, bw_mhz, num_rb):
    return NsNrMacScheduler.ComputeThroughputMbps(sinr, alloc_rb, layers, cqi, freq_ghz, bw_mhz, num_rb)

def compute_latency(slice_type, sinr, dist, active_ues=1, total_rbs=None, scs_hz=None):
    return NsLteLatencyModel.ComputeLatencyMs(
        slice_type, sinr, dist, active_ues,
        total_rbs or CFG["NUM_RB"], scs_hz or int(CFG["SCS_HZ"]))

                                                               
               
                                                               
SBA_MAX_EVENTS = 120

class SBABus:
    def __init__(self):
        self._lock   = threading.Lock()
        self._events = collections.deque(maxlen=SBA_MAX_EVENTS)
        self._seq    = 0

    def emit(self, nf, direction, interface, event, payload=None):
        with self._lock:
            self._seq += 1
            self._events.appendleft({
                "seq":     self._seq,
                "ts":      round(time.time() * 1000),
                "nf":      nf, "dir": direction,
                "iface":   interface, "event": event,
                "payload": payload or {},
            })

    def snapshot(self, n=40):
        with self._lock:
            return list(self._events)[:n]

sba = SBABus()

                                                               
          
                                                               
class UE:
    def __init__(self, uid):
        self.id             = uid
        self.active         = False
        self.manually_placed= False
        self.x = self.y     = 0.0
        self.dist_to_gnb    = 0.0                                                   
        self.speed          = 0.0
        self.heading        = 0.0
        self.is_los         = False
        self.slice          = "EMBB"
        self.req_rate       = 10.0
        self.gbr            = False
        self.shadow_db = self.fast_db = self.doppler = 0.0
        self.sinr = self.rsrp = self.rsrq = 0.0
        self.cqi = self.mod_order = 0
        self.layers  = 1
        self.alloc_rb= 0
        self.tput = self.latency = self.bler = 0.0
        self.pf_metric = self.avg_tput = 1.0
        self.in_ho = False; self.ho_timer = 0
        self.tx_bytes = self.rx_bytes = self.harq_fails = 0
        self.nas_state = "DEREG"; self.nas_timer = 0
        self.rand_challenge = 0; self.usim_sqn = 1
        self.gnb_id  = 0
        self.gnb_x   = 0.0                                                 
        self.gnb_y   = 0.0
        self.pdu_active = False; self.pdu_ip = ""

    def randomize(self, gnb_positions=None, radius=None, fix_speed=None, fix_heading=None):
        """
        Spawn UE near a randomly chosen gNB (if multiple exist),
        ensuring equal load distribution across all cells from birth.
        """
        radius = radius or CFG["CELL_RADIUS_M"]
                                         
        if gnb_positions and len(gnb_positions) > 0:
            gx, gy = random.choice(gnb_positions)
        else:
            gx, gy = 0.0, 0.0
        r   = random.uniform(10.0, radius)
        phi = random.uniform(0, 2*math.pi)
        self.x = gx + r*math.cos(phi)
        self.y = gy + r*math.sin(phi)
        self.dist_to_gnb = r
        wfx = weather_fx()
        if fix_speed is not None:
            self.speed = fix_speed
        else:
            roll = random.random()
            base = 0.5 if roll < 0.60 else (5.0 if roll < 0.85 else 0.0)
            top  = 3.0 if roll < 0.60 else (20.0 if roll < 0.85 else 0.0)
            self.speed = random.uniform(base, top) * wfx["speed_scale"] if top > 0 else 0.0
        self.heading   = fix_heading if fix_heading is not None else random.uniform(0, 2*math.pi)
        self.is_los    = self.dist_to_gnb < 100.0
        self.shadow_db = random.gauss(0, 4.0)
        self.fast_db   = fast_fading(self.speed, self.is_los)
        sr = random.random()
        if   sr < 0.10: self.slice = "URLLC"; self.req_rate = 1.0;  self.gbr = True
        elif sr < 0.25: self.slice = "MMTC";  self.req_rate = 0.1;  self.gbr = False
        else:           self.slice = "EMBB";  self.req_rate = random.uniform(5,100); self.gbr = False
        self.layers = 1; self.pf_metric = self.avg_tput = 1.0
        self.active = True
        self.tx_bytes = self.rx_bytes = self.harq_fails = 0
        self.in_ho = False; self.ho_timer = 0
        self.pdu_active = False
        self.nas_state = "DEREG"; self.nas_timer = 1
        self.usim_sqn  = 1; self.gnb_id = 0
        self.manually_placed = False

    def to_dict(self):
        return {
            "id": self.id, "x": round(self.x,2), "y": round(self.y,2),
            "dist": round(self.dist_to_gnb,1),
            "slice": SLICE_NAMES.get(self.slice, self.slice),
            "nas": self.nas_state,
            "sinr": round(self.sinr,2), "cqi": self.cqi,
            "layers": self.layers,
            "tput": round(self.tput,2), "latency": round(self.latency,2),
            "bler": round(self.bler,4), "rsrp": round(self.rsrp,1),
            "rsrq": round(self.rsrq,2),
            "doppler": round(self.doppler,1),
            "speed": round(self.speed,2),
            "mod": MOD_NAMES.get(self.mod_order, "OFF"),
            "ho": self.in_ho, "manual": self.manually_placed,
            "gnbId": self.gnb_id,
            "pduIp": self.pdu_ip,
            "allocRb": self.alloc_rb,
            "txBytes": self.tx_bytes, "rxBytes": self.rx_bytes,
            "harqFails": self.harq_fails,
        }

                                                               
           
                                                               
class GNB:
    def __init__(self, gid, x, y, tx_power=46.0, label="gNB-0",
                 height=30.0, freq_ghz=28.0, bw_mhz=400.0,
                 tx_ant=64, num_rb=None,
                 rb_embb=None, rb_urllc=None, rb_mmtc=None,
                 max_layers=4):
        self.id       = gid
        self.x        = x;  self.y = y
        self.tx_power = tx_power
        self.label    = label
        self.height   = height
        self.freq_ghz = freq_ghz
        self.bw_mhz   = bw_mhz
        self.tx_ant   = tx_ant
        self.max_layers = max_layers
                                                    
        self.num_rb   = num_rb if num_rb is not None else num_rb_from_bw(bw_mhz, freq_ghz)
                                                                                 
        total = self.num_rb
        if rb_embb is None and rb_urllc is None and rb_mmtc is None:
                                    
            self.rb_urllc = max(1, round(total * 0.10))
            self.rb_mmtc  = max(1, round(total * 0.10))
            self.rb_embb  = total - self.rb_urllc - self.rb_mmtc
        else:
            self.rb_embb  = rb_embb  if rb_embb  is not None else max(1, total - 54)
            self.rb_urllc = rb_urllc if rb_urllc is not None else 27
            self.rb_mmtc  = rb_mmtc  if rb_mmtc  is not None else 27
        self.active   = True
                                    
        self.e2_avg_sinr = 0.0
        self.e2_avg_bler = 0.0
        self.e2_load     = 0.0
        self.e2_tick     = 0

    def recalc_rb(self):
        """Re-derive num_rb whenever bw_mhz or freq_ghz changes."""
        self.num_rb = num_rb_from_bw(self.bw_mhz, self.freq_ghz)
                                    
        total = self.num_rb
        self.rb_urllc = max(1, round(total * 0.10))
        self.rb_mmtc  = max(1, round(total * 0.10))
        self.rb_embb  = total - self.rb_urllc - self.rb_mmtc

    def to_dict(self):
        return {
            "id": self.id, "x": round(self.x,1), "y": round(self.y,1),
            "txPower": self.tx_power, "label": self.label,
            "height": self.height, "freqGhz": self.freq_ghz,
            "bwMhz": self.bw_mhz, "txAnt": self.tx_ant,
            "numRb": self.num_rb, "active": self.active,
            "rbEmbb": self.rb_embb, "rbUrllc": self.rb_urllc, "rbMmtc": self.rb_mmtc,
            "maxLayers": self.max_layers,
            "e2AvgSinr": round(self.e2_avg_sinr, 2),
            "e2AvgBler": round(self.e2_avg_bler, 4),
            "e2Load":    round(self.e2_load, 3),
        }

                                                               
                   
                                                               
def amf_process_nas(ue, gnb=None):
    """NAS state machine — also fires PCAP events for each state transition."""
    if ue.nas_timer > 0: ue.nas_timer -= 1; return
    if ue.nas_state == "DEREG":
        ue.rand_challenge = random.getrandbits(64)
        ue.nas_state = "AUTH?"; ue.nas_timer = 1
        if gnb: pcap_nas_registration(ue, gnb)
    elif ue.nas_state == "AUTH?":
        ue.usim_sqn += 1
        ue.nas_state = "AUTH_OK"; ue.nas_timer = 1
        if gnb: pcap_nas_auth(ue, gnb)
    elif ue.nas_state == "AUTH_OK":
        ue.nas_state = "REG"; ue.nas_timer = 1
        if gnb: pcap_rrc_setup(ue, gnb)
    elif ue.nas_state == "REG":
        ue.nas_state = "PDU"; ue.nas_timer = 2
    elif ue.nas_state == "PDU":
        ue.pdu_active = True
        ip = 0x0A000000 | (ue.id & 0xFFFF)
        ue.pdu_ip = f"{(ip>>24)&0xFF}.{(ip>>16)&0xFF}.{(ip>>8)&0xFF}.{ip&0xFF}"
        ue.nas_state = "UP"; ue.nas_timer = 0
        if gnb: pcap_pdu_session(ue, gnb)
    elif ue.nas_state == "UP":
        ue.nas_timer = 300

                                                               
                                         
                                                               
def update_mobility(ue):
    """Move UE; clamp to CELL_RADIUS_M from its current serving gNB."""
    if ue.manually_placed or ue.speed < 0.01: return
    wfx = weather_fx()
    ue.heading += random.gauss(0, 0.3 * wfx["speed_scale"])
    ue.x += ue.speed * math.cos(ue.heading)
    ue.y += ue.speed * math.sin(ue.heading)
                                                             
    dx = ue.x - ue.gnb_x; dy = ue.y - ue.gnb_y
    nd = math.sqrt(dx*dx + dy*dy)
    if nd > CFG["CELL_RADIUS_M"]:
        ue.heading += math.pi + random.gauss(0, 0.1)
        scale = CFG["CELL_RADIUS_M"] / nd * 0.95
        ue.x = ue.gnb_x + dx*scale
        ue.y = ue.gnb_y + dy*scale
    ue.dist_to_gnb = math.sqrt((ue.x - ue.gnb_x)**2 + (ue.y - ue.gnb_y)**2)
    ue.is_los = ue.dist_to_gnb < 100.0

def update_phy(ue, gnb, interferer_positions, active_ues=1):
    """
    Full PHY computation for UE served by 'gnb'.
    Uses the UE's ACTUAL distance to gnb (not distance from world origin).
    """
    tx_dbm   = gnb.tx_power
    freq_ghz = gnb.freq_ghz
    bw_mhz   = gnb.bw_mhz
    tx_ant   = gnb.tx_ant
    rx_ant   = CFG["UE_RX_ANT"]
    gnb_h    = gnb.height

    ue.shadow_db = update_shadow(ue.shadow_db, ue.speed)
    ue.fast_db   = fast_fading(ue.speed, ue.is_los)
    ue.doppler   = doppler_hz(ue.speed, freq_ghz)

                                  
    dist = ue.dist_to_gnb

                                                                         
    max_layers = gnb.max_layers
    ue.sinr = compute_sinr(dist, ue.x, ue.y, ue.shadow_db, ue.fast_db,
                           1,                                        
                           tx_dbm, freq_ghz, bw_mhz, tx_ant, rx_ant, gnb_h,
                           interferer_positions, active_ues=active_ues)
    ue.sinr = max(-15.0, min(35.0, ue.sinr))

    if   ue.sinr > 20.0: ue.layers = min(4, max_layers)
    elif ue.sinr > 10.0: ue.layers = min(2, max_layers)
    else:                ue.layers = 1

                                                                    
    ue.sinr = compute_sinr(dist, ue.x, ue.y, ue.shadow_db, ue.fast_db,
                           ue.layers,
                           tx_dbm, freq_ghz, bw_mhz, tx_ant, rx_ant, gnb_h,
                           interferer_positions, active_ues=active_ues)
    ue.sinr = max(-15.0, min(35.0, ue.sinr))

    ue.cqi       = sinr_to_cqi(ue.sinr)
    ue.mod_order = CQI_TABLE[ue.cqi][0] if ue.cqi > 0 else 0
    pl           = umi_path_loss(dist, freq_ghz, gnb_h)
    ue.rsrp      = tx_dbm - pl - ue.shadow_db + ue.fast_db
    noise_pow    = CFG["THERMAL_DBM_HZ"] + CFG["NOISE_FIG_DB"] + 10*math.log10(bw_mhz*1e6)
    tot          = 10*math.log10(10**(ue.rsrp/10) + 10**(noise_pow/10))
    ue.rsrq      = ue.rsrp - tot
    ue.bler      = ldpc_bler(ue.sinr, ue.cqi)

                                        
    if ue.rsrp < -110.0 and not ue.in_ho:
        ue.in_ho = True; ue.ho_timer = 3
    if ue.in_ho:
        if ue.ho_timer > 0:
            ue.ho_timer -= 1
        else:
            ue.in_ho = False
                                                              
            ue.x = ue.gnb_x + (ue.x - ue.gnb_x)*0.3
            ue.y = ue.gnb_y + (ue.y - ue.gnb_y)*0.3
            ue.dist_to_gnb = math.sqrt((ue.x-ue.gnb_x)**2 + (ue.y-ue.gnb_y)**2)
            ue.is_los = ue.dist_to_gnb < 100.0
            ue.nas_state = "DEREG"; ue.nas_timer = 1; ue.pdu_active = False

def update_mac(ue, gnb, active_ues=1):
    """Compute throughput and latency using gNB's actual radio parameters."""
    if ue.in_ho or ue.alloc_rb == 0 or ue.nas_state != "UP":
        ue.tput    = 0.0
        scs        = nr_scs_hz(gnb.freq_ghz)
        ue.latency = 50.0 if ue.in_ho else compute_latency(
                        ue.slice, ue.sinr, ue.dist_to_gnb,
                        active_ues, gnb.num_rb, scs)
        return
    raw = compute_tput(ue.sinr, ue.alloc_rb, ue.layers, ue.cqi,
                       gnb.freq_ghz, gnb.bw_mhz, gnb.num_rb)
    ue.tput    = raw
    scs        = nr_scs_hz(gnb.freq_ghz)
    ue.latency = compute_latency(ue.slice, ue.sinr, ue.dist_to_gnb,
                                 active_ues, gnb.num_rb, scs)
    if random.random() < ue.bler: ue.harq_fails += 1
    b = int(ue.tput * 1e6 / 8.0)
    ue.rx_bytes += b; ue.tx_bytes += b // 10
    ue.avg_tput  = 0.1*ue.tput + 0.9*ue.avg_tput
    ue.pf_metric = ue.tput/ue.avg_tput if ue.avg_tput > 0 else 1.0

def run_scheduler(ues, gnb):
    """
    Schedule RBs to UEs using gNB's own rb_embb/rb_urllc/rb_mmtc split.
    Supports PF, RR, MaxCQI.
    """
    rb_embb  = gnb.rb_embb
    rb_urllc = gnb.rb_urllc
    rb_mmtc  = gnb.rb_mmtc
    sched    = CFG["SCHEDULER"]
    buckets  = {"EMBB": [], "URLLC": [], "MMTC": []}
    for u in ues:
        buckets[u.slice].append(u)
    quotas = {"EMBB": rb_embb, "URLLC": rb_urllc, "MMTC": rb_mmtc}
    for sl, lst in buckets.items():
        if not lst: continue
        if   sched == "PF":     lst.sort(key=lambda u: -u.pf_metric)
        elif sched == "MAXCQI": lst.sort(key=lambda u: -u.cqi)
        else:                   random.shuffle(lst)
        rpu = quotas[sl] // len(lst)
        lo  = quotas[sl] - rpu*len(lst)
        for i, u in enumerate(lst):
            u.alloc_rb = rpu + (lo if i == 0 else 0)

def compute_net_metrics(ues):
    if not ues:
        return {}
    m = dict(
        activeUes=0, connectedUes=0, authUes=0,
        totalTput=0, avgSinr=0, avgLatency=0, avgBler=0,
        avgRsrp=0, avgDoppler=0, usedRb=0,
        embbUes=0, urllcUes=0, mmtcUes=0,
        embbTput=0, urllcTput=0, mmtcTput=0,
        embbLatency=0, urllcLatency=0, mmtcLatency=0,
    )
    for u in ues:
        m["activeUes"] += 1
        if u.nas_state == "UP": m["connectedUes"] += 1
        else:                   m["authUes"] += 1
        m["totalTput"]  += u.tput
        m["avgSinr"]    += u.sinr
        m["avgLatency"] += u.latency
        m["avgBler"]    += u.bler
        m["avgRsrp"]    += u.rsrp
        m["avgDoppler"] += u.doppler
        m["usedRb"]     += u.alloc_rb
        if u.slice == "EMBB":
            m["embbUes"]+=1; m["embbTput"]+=u.tput; m["embbLatency"]+=u.latency
        elif u.slice == "URLLC":
            m["urllcUes"]+=1; m["urllcTput"]+=u.tput; m["urllcLatency"]+=u.latency
        elif u.slice == "MMTC":
            m["mmtcUes"]+=1; m["mmtcTput"]+=u.tput; m["mmtcLatency"]+=u.latency
    n = m["activeUes"]
    for k in ["avgSinr","avgLatency","avgBler","avgRsrp","avgDoppler"]:
        m[k] /= n
                                                                   
    m["specEffic"] = m["totalTput"]*1e6 / (CFG["BW_MHZ"]*1e6)
    m["rbUtil"]    = m["usedRb"] / max(1, CFG["NUM_RB"])
    if m["embbUes"]  > 0: m["embbLatency"]  /= m["embbUes"]
    if m["urllcUes"] > 0: m["urllcLatency"] /= m["urllcUes"]
    if m["mmtcUes"]  > 0: m["mmtcLatency"]  /= m["mmtcUes"]
    for k, v in m.items():
        if isinstance(v, float): m[k] = round(v, 4)
    return m
                                                               
                                                   
                                                               
class RealisticSim:
    """
    Single-cell real network (gNB-0 at origin).
    Used for NWDAF/NEF data ingestion into twin.
    """
    def __init__(self):
        self.gnb0   = GNB(0, 0.0, 0.0,
                          CFG["GNB_TX_DBM"], "gNB-0-Real",
                          CFG["GNB_HEIGHT_M"], CFG["FREQ_GHZ"],
                          CFG["BW_MHZ"], CFG["GNB_TX_ANT"])
        self.pool   = [UE(i) for i in range(POOL_SIZE_REAL)]
        self.active = set()
        self.lock   = threading.Lock()
        self.metrics= {}
        self._next_drop = random.randint(5,15)
        self._next_join = random.randint(5,15)
        self._tick  = 0
        init = (TARGET_MIN_REAL + TARGET_MAX_REAL) // 2
        for i in range(init):
            self.pool[i].randomize(gnb_positions=[(0.0,0.0)])
            self.pool[i].gnb_x = 0.0; self.pool[i].gnb_y = 0.0
            self.active.add(i)

    def _drop(self):
        with self.lock:
            if len(self.active) <= TARGET_MIN_REAL: return
            av = list(self.active); random.shuffle(av)
            for k in range(min(random.randint(1,3), len(av))):
                if len(self.active) <= TARGET_MIN_REAL: break
                self.pool[av[k]].active = False; self.active.discard(av[k])

    def _join(self):
        with self.lock:
            if len(self.active) >= TARGET_MAX_REAL: return
            avail = [i for i in range(POOL_SIZE_REAL) if not self.pool[i].active]
            if not avail: return
            random.shuffle(avail)
            for k in range(min(random.randint(1,3), len(avail))):
                if len(self.active) >= TARGET_MAX_REAL: break
                self.pool[avail[k]].randomize(gnb_positions=[(0.0,0.0)])
                self.pool[avail[k]].gnb_x = 0.0
                self.pool[avail[k]].gnb_y = 0.0
                self.active.add(avail[k])

    def step(self):
        g = self.gnb0
        with self.lock:
            ues = [self.pool[i] for i in self.active]
        n = len(ues)
                                                              
        isd = CFG["ISD_M"]
        hex_nbrs = [(isd*math.cos(i*math.pi/3), isd*math.sin(i*math.pi/3)) for i in range(6)]
        for u in ues:
            u.gnb_x = 0.0; u.gnb_y = 0.0
            update_mobility(u)
        for u in ues:
            update_phy(u, g, hex_nbrs, active_ues=n)
        for u in ues:
            amf_process_nas(u, g)
        run_scheduler(ues, g)
        for u in ues:
            update_mac(u, g, active_ues=n)
                                                    
        for u in ues:
            if u.nas_state == "UP" and u.pdu_active:
                pcap_data_plane(u, g, self._tick)
        m = compute_net_metrics(ues)
        with self.lock:
            self.metrics = m
        self._tick += 1
        if self._tick >= self._next_drop:
            self._drop(); self._next_drop = self._tick + random.randint(5,15)
        if self._tick >= self._next_join:
            self._join(); self._next_join = self._tick + random.randint(5,15)

    def get_metrics(self):
        with self.lock: return dict(self.metrics)

    def run(self, running):
        while running[0]:
            t0 = time.time(); self.step()
            time.sleep(max(0, CFG["TICK_MS"]/1000.0 - (time.time()-t0)))

                                                               
                
                                                               
class NWDAF:
    def __init__(self, real_sim):
        self.real = real_sim
        self._cache = {}

    def collect_analytics(self):
        m = self.real.get_metrics()
        analytics = {
            "nf": "NWDAF",
            "analytics": {
                "loadLevel":      m.get("rbUtil", 0),
                "congestion":     m.get("rbUtil", 0) > 0.85,
                "avgSinr":        m.get("avgSinr", 0),
                "avgBler":        m.get("avgBler", 0),
                "totalTput_Mbps": m.get("totalTput", 0),
                "activeUes":      m.get("activeUes", 0),
                "connectedUes":   m.get("connectedUes", 0),
                "sliceLoad": {
                    "EMBB":  {"ues": m.get("embbUes",0),  "tput": m.get("embbTput",0)},
                    "URLLC": {"ues": m.get("urllcUes",0), "tput": m.get("urllcTput",0)},
                    "MMTC":  {"ues": m.get("mmtcUes",0),  "tput": m.get("mmtcTput",0)},
                },
            }
        }
        self._cache = analytics
        sba.emit("NWDAF", "IN", "Nnwdaf", "AnalyticsInfo.Response",
                 {"ues": analytics["analytics"]["activeUes"],
                  "load": round(analytics["analytics"]["loadLevel"], 3),
                  "congestion": analytics["analytics"]["congestion"]})
        return analytics

class NEF:
    def __init__(self, real_sim):
        self.real = real_sim

    def get_exposure_events(self):
        m = self.real.get_metrics()
        events = []
        if m.get("rbUtil", 0) > 0.80:
            events.append({"eventId": "CONGESTION_INFO", "congLevel": "HIGH",
                           "rbUtil": round(m["rbUtil"], 3)})
        if m.get("avgBler", 0) > 0.12:
            events.append({"eventId": "SLICE_QOS", "violation": "HIGH_BLER",
                           "avgBler": round(m["avgBler"], 4)})
        if m.get("connectedUes", 0) > 0:
            events.append({"eventId": "UE_REACHABILITY",
                           "connectedUes": m["connectedUes"]})
        if events:
            sba.emit("NEF", "IN", "Nnef", "EventExposure.Notify",
                     {"events": [e["eventId"] for e in events]})
        return events

class NearRTRIC:
    def __init__(self):
        self._last_telemetry = {}

    def process_e2_telemetry(self, gnbs, ues):
        recs = {}
        for gnb in gnbs:
            gnb_ues = [u for u in ues if u.gnb_id == gnb.id]
            if not gnb_ues: continue
            avg_sinr = sum(u.sinr for u in gnb_ues) / len(gnb_ues)
            avg_bler = sum(u.bler for u in gnb_ues) / len(gnb_ues)
            load     = sum(u.alloc_rb for u in gnb_ues) / max(1, gnb.num_rb)
            gnb.e2_avg_sinr = avg_sinr
            gnb.e2_avg_bler = avg_bler
            gnb.e2_load     = load
            gnb.e2_tick     += 1
            recs[gnb.id] = {
                "gnbId":   gnb.id,
                "avgSinr": round(avg_sinr, 2),
                "avgBler": round(avg_bler, 4),
                "load":    round(load, 3),
                "ues":     len(gnb_ues),
            }
        if recs:
            sba.emit("Near-RT RIC", "IN", "E2AP", "RIC_INDICATION",
                     {"gNBs": len(recs)})
        self._last_telemetry = recs
        return recs

    def get_recommendations(self):
        return dict(self._last_telemetry)

class UPFMonitor:
    def __init__(self, real_sim):
        self.real = real_sim
        self.upf_load    = 0.0
        self.sampled_bps = 0.0

    def sample(self):
        m = self.real.get_metrics()
        total_tput = m.get("totalTput", 0)
        self.sampled_bps = total_tput * 1e6 * random.uniform(0.95, 1.05)
        self.upf_load    = min(1.0, total_tput / max(1.0, CFG["BW_MHZ"] * 2))
        report = {
            "sampledBps": round(self.sampled_bps / 1e6, 2),
            "upfLoad":    round(self.upf_load, 3),
            "congested":  self.upf_load > 0.75,
        }
        if self.upf_load > 0.75:
            sba.emit("UPF", "IN", "N4/N9", "TrafficSample",
                     {"load": round(self.upf_load, 3), "congested": True})
        return report

                                                               
                        
                                                               
def _clone_ue(src, dst_id):
    u = UE(dst_id)
    for attr in ["x","y","dist_to_gnb","speed","heading","is_los","shadow_db","fast_db","doppler",
                 "sinr","rsrp","rsrq","cqi","mod_order","layers","alloc_rb","tput","latency",
                 "bler","slice","req_rate","gbr","nas_state","nas_timer","rand_challenge",
                 "usim_sqn","gnb_id","gnb_x","gnb_y","pdu_active","pdu_ip","pf_metric",
                 "avg_tput","in_ho","ho_timer","tx_bytes","rx_bytes","harq_fails"]:
        setattr(u, attr, getattr(src, attr))
    u.active = True; u.manually_placed = False
    return u

class TwinSim:
    def __init__(self, real_sim):
        self.real    = real_sim
        self.lock    = threading.Lock()
        self.metrics = {}
        self.ues_snapshot = []
        self._tick   = 0

        self.nwdaf = NWDAF(real_sim)
        self.nef   = NEF(real_sim)
        self.ric   = NearRTRIC()
        self.upf   = UPFMonitor(real_sim)

                                                         
        self.gnbs = [GNB(0, 0.0, 0.0,
                         CFG["GNB_TX_DBM"], "gNB-0",
                         CFG["GNB_HEIGHT_M"], CFG["FREQ_GHZ"],
                         CFG["BW_MHZ"], CFG["GNB_TX_ANT"])]

                                                          
        self._hex_interferers = []
        self._rebuild_hex()

                 
        self.pool  = [UE(i) for i in range(POOL_SIZE_TWIN)]
        self.active = set()
        self.extra  = set()

        with self.real.lock:
            for rid in self.real.active:
                src = self.real.pool[rid]
                if rid < POOL_SIZE_TWIN:
                    self.pool[rid] = _clone_ue(src, rid)
                    self.active.add(rid)

    def _rebuild_hex(self):
        isd = CFG["ISD_M"]
        self._hex_interferers = [
            (isd*math.cos(i*math.pi/3), isd*math.sin(i*math.pi/3))
            for i in range(6)
        ]

    def _all_interferer_positions(self, serving_gnb_id):
        """Return positions of all gNBs EXCEPT the serving one + hex background."""
        other_gnbs = [(g.x, g.y) for g in self.gnbs if g.active and g.id != serving_gnb_id]
        return self._hex_interferers + other_gnbs

    def _gnb_positions(self):
        return [(g.x, g.y) for g in self.gnbs if g.active]

    def _reconcile_ues(self):
        with self.real.lock:
            real_active = set(self.real.active)
            real_pool   = self.real.pool
        to_add = real_active - self.active
        for rid in to_add:
            if rid >= POOL_SIZE_TWIN: continue
            self.pool[rid] = _clone_ue(real_pool[rid], rid)
            self.active.add(rid)
        to_remove = self.active - real_active
        for rid in to_remove:
            if self.pool[rid].manually_placed: continue
            self.pool[rid].active = False; self.active.discard(rid)
        if to_add or to_remove:
            sba.emit("AMF/SMF", "IN", "Namf/Nsmf", "UE_STATE_SYNC",
                     {"added": len(to_add), "removed": len(to_remove)})

    def step(self):
                                           
        self.nwdaf.collect_analytics()
        self.nef.get_exposure_events()
        self.upf.sample()
        self._reconcile_ues()

        with self.lock:
            all_active  = self.active | self.extra
            ues         = [self.pool[i] for i in all_active if self.pool[i].active]
            gnbs_copy   = [g for g in self.gnbs if g.active]

        num_gnbs       = max(1, len(gnbs_copy))
        system_capacity= 250 * num_gnbs
        gnb_positions  = [(g.x, g.y) for g in gnbs_copy]

                                                                              
        for u in ues:
            best_gnb = gnbs_copy[0]
            best_d   = 1e18
            for g in gnbs_copy:
                dx, dy = u.x - g.x, u.y - g.y
                d = math.sqrt(dx*dx + dy*dy)
                if d < best_d:
                    best_d = d; best_gnb = g
            u.gnb_id    = best_gnb.id
            u.gnb_x     = best_gnb.x
            u.gnb_y     = best_gnb.y
            u.dist_to_gnb = best_d                                

                                 
        for u in ues:
            update_mobility(u)

                                                           
        for u in ues:
                                                                                
            best_gnb = gnbs_copy[0]; best_d = 1e18
            for g in gnbs_copy:
                dx, dy = u.x - g.x, u.y - g.y
                d = math.sqrt(dx*dx + dy*dy)
                if d < best_d:
                    best_d = d; best_gnb = g
            u.gnb_id      = best_gnb.id
            u.gnb_x       = best_gnb.x
            u.gnb_y       = best_gnb.y
            u.dist_to_gnb = best_d

                                                       
        gnb_ue_map = {g.id: [] for g in gnbs_copy}
        for u in ues:
            gnb_ue_map[u.gnb_id].append(u)

        for gnb in gnbs_copy:
            gnb_ues = gnb_ue_map[gnb.id]
            if not gnb_ues: continue
            intf_pos = self._all_interferer_positions(gnb.id)
            for u in gnb_ues:
                update_phy(u, gnb, intf_pos, active_ues=len(gnb_ues))

                                                          
        for u in ues:
                                                        
            serving_gnb = next((g for g in gnbs_copy if g.id == u.gnb_id), gnbs_copy[0])
            amf_process_nas(u, serving_gnb)

                                                                            
        for gnb in gnbs_copy:
            gnb_ues = gnb_ue_map[gnb.id]
            if not gnb_ues: continue
            run_scheduler(gnb_ues, gnb)
            for u in gnb_ues:
                update_mac(u, gnb, active_ues=len(gnb_ues))

                                                                           
        _pcap_budget = 20                                                 
        _pcap_written = 0
        for gnb in gnbs_copy:
            gnb_ues = gnb_ue_map[gnb.id]
            for u in gnb_ues:
                if _pcap_written >= _pcap_budget: break
                if u.nas_state == "UP" and u.pdu_active:
                    pcap_data_plane(u, gnb, self._tick)
                    _pcap_written += 1
                                        
                    if u.in_ho and u.ho_timer == 2:
                                                                         
                        others = [g for g in gnbs_copy if g.id != gnb.id]
                        if others:
                            pcap_handover(u, gnb, random.choice(others))

        m = compute_net_metrics(ues)
        m["totalGnbs"]      = num_gnbs
        m["systemCapacity"] = system_capacity

                                                        
        self.ric.process_e2_telemetry(gnbs_copy, ues)
        if self._tick % 5 == 0:
            for gnb in gnbs_copy:
                pcap_ric_e2(gnb, gnb_ue_map[gnb.id], self._tick)
                                                     
        if self._tick % 10 == 0:
            analytics = self.nwdaf._cache.get("analytics", {})
            if analytics:
                pcap_sba_nwdaf(analytics)
                                           
        if self._tick % 30 == 0:
            threading.Thread(target=pcap_engine.flush_all, daemon=True).start()

        with self.lock:
            self.metrics      = m
            self.ues_snapshot = [u.to_dict() for u in ues]
        self._tick += 1

    def get_status(self):
        with self.lock:
            m         = dict(self.metrics)
            ues_snap  = list(self.ues_snapshot)
            gnbs_list = [g.to_dict() for g in self.gnbs]

        slices = {"eMBB":{"tput":[],"lat":[]}, "URLLC":{"tput":[],"lat":[]}, "mMTC":{"tput":[],"lat":[]}}
        for u in ues_snap:
            sl = u.get("slice","eMBB")
            if sl in slices:
                slices[sl]["tput"].append(u.get("tput",0))
                slices[sl]["lat"].append(u.get("latency",0))
        slice_stats = {}
        for sl, d in slices.items():
            slice_stats[sl] = {
                "avgTput":    round(sum(d["tput"])/len(d["tput"]),2) if d["tput"] else 0,
                "avgLatency": round(sum(d["lat"])/len(d["lat"]),2)   if d["lat"]  else 0,
                "ues": len(d["tput"]),
            }
        m["sliceStats"] = slice_stats

        return {
            "twin":  m,
            "real":  self.real.get_metrics(),
            "ues":   ues_snap,
            "gnbs":  gnbs_list,
            "cfg":   dict(CFG),
            "sba": {
                "events":        sba.snapshot(40),
                "ric_telemetry": self.ric.get_recommendations(),
                "upf": {
                    "load":        round(self.upf.upf_load, 3),
                    "sampledMbps": round(self.upf.sampled_bps/1e6, 2),
                },
            },
        }

    def get_metrics(self):
        with self.lock: return dict(self.metrics)

                                                               
    def add_gnb(self, x, y, tx_power, label, height, freq_ghz, bw_mhz,
                tx_ant, num_rb=None, rb_embb=None, rb_urllc=None, rb_mmtc=None,
                max_layers=4):
        with self.lock:
            gid = max((g.id for g in self.gnbs), default=-1) + 1
            if not label: label = f"gNB-{gid}"
            g = GNB(gid, x, y, tx_power, label, height, freq_ghz, bw_mhz,
                    tx_ant, num_rb, rb_embb, rb_urllc, rb_mmtc, max_layers)
            self.gnbs.append(g)
            self._rebuild_hex()
            return gid

    def update_gnb(self, gid, params):
        with self.lock:
            for g in self.gnbs:
                if g.id == gid:
                    for k, v in params.items():
                        if hasattr(g, k): setattr(g, k, v)
                                                                          
                    if "freq_ghz" in params or "bw_mhz" in params:
                        g.recalc_rb()
                    self._rebuild_hex()
                    return True
        return False

    def delete_gnb(self, gid):
        with self.lock:
            self.gnbs = [g for g in self.gnbs if g.id != gid]
            if not self.gnbs:
                self.gnbs = [GNB(0, 0.0, 0.0, CFG["GNB_TX_DBM"], "gNB-0",
                                 CFG["GNB_HEIGHT_M"], CFG["FREQ_GHZ"],
                                 CFG["BW_MHZ"], CFG["GNB_TX_ANT"])]
            self._rebuild_hex()
            return True

    def add_ue(self, count, slice_name, fix_x=None, fix_y=None,
               fix_speed=None, fix_heading=None):
        slice_map = {"embb":"EMBB","urllc":"URLLC","mmtc":"MMTC",
                     "eMBB":"EMBB","URLLC":"URLLC","mMTC":"MMTC",
                     "EMBB":"EMBB","MMTC":"MMTC"}
        sl = slice_map.get(slice_name, "EMBB")
        added = 0
        with self.lock:
            gnb_positions = [(g.x, g.y) for g in self.gnbs if g.active]
            needed = POOL_SIZE_REAL + len(self.extra) + count + 10
            while len(self.pool) < needed:
                self.pool.append(UE(len(self.pool)))
            for i in range(POOL_SIZE_REAL, len(self.pool)):
                if added >= count: break
                if not self.pool[i].active:
                    self.pool[i].randomize(gnb_positions=gnb_positions,
                                           fix_speed=fix_speed, fix_heading=fix_heading)
                    self.pool[i].slice = sl
                    if fix_x is not None and fix_y is not None:
                        self.pool[i].x = fix_x; self.pool[i].y = fix_y
                        self.pool[i].manually_placed = True
                    self.extra.add(i); added += 1
        return added

    def move_ue(self, uid, x, y):
        with self.lock:
            if uid < len(self.pool) and self.pool[uid].active:
                u = self.pool[uid]
                u.x = x; u.y = y
                u.manually_placed = True
                return True
        return False

    def remove_ue(self, uid):
        with self.lock:
            if uid < len(self.pool) and self.pool[uid].active:
                self.pool[uid].active = False
                self.active.discard(uid); self.extra.discard(uid)
                return True
        return False

    def set_param(self, key, value):
        with self.lock:
            if key in CFG:
                CFG[key] = type(CFG[key])(value)
                                                         
                if key == "ISD_M": self._rebuild_hex()
                return True
            lmap = {"txPower":"GNB_TX_DBM","freqGhz":"FREQ_GHZ",
                    "bwMhz":"BW_MHZ","noiseFig":"NOISE_FIG_DB"}
            if key in lmap:
                CFG[lmap[key]] = float(value)
                return True
        return False

    def set_gnb_param(self, gid, key, value):
        with self.lock:
            for g in self.gnbs:
                if g.id == gid and hasattr(g, key):
                    val = type(getattr(g, key))(value)
                    setattr(g, key, val)
                    if key in ("freq_ghz", "bw_mhz"):
                        g.recalc_rb()
                    self._rebuild_hex()
                    return True
        return False

    def tweak_rb(self, gid, embb, urllc, mmtc):
        """Set per-gNB RB split (or global if gid=-1)."""
        with self.lock:
            if gid == -1:
                                                  
                for g in self.gnbs:
                    total = g.num_rb
                    frac_e = embb / max(1, embb+urllc+mmtc)
                    frac_u = urllc / max(1, embb+urllc+mmtc)
                    g.rb_urllc = max(1, round(total * frac_u))
                    g.rb_mmtc  = max(1, round(total * (mmtc/(embb+urllc+mmtc))))
                    g.rb_embb  = total - g.rb_urllc - g.rb_mmtc
                CFG["RB_EMBB"]  = embb
                CFG["RB_URLLC"] = urllc
                CFG["RB_MMTC"]  = mmtc
            else:
                for g in self.gnbs:
                    if g.id == gid:
                        g.rb_embb = embb; g.rb_urllc = urllc; g.rb_mmtc = mmtc

    def reset(self):
        with self.lock:
            for k, v in [("GNB_TX_DBM",46.0),("FREQ_GHZ",28.0),("BW_MHZ",400.0),
                         ("NOISE_FIG_DB",7.0),("RB_EMBB",216),("RB_URLLC",27),
                         ("RB_MMTC",27),("WEATHER","NORMAL"),("SCHEDULER","PF"),
                         ("GNB_TX_ANT",64),("UE_RX_ANT",4),("MAX_LAYERS",4),
                         ("NUM_RB",270)]:
                CFG[k] = v
            for i in list(self.extra):
                self.pool[i].active = False
            self.extra.clear()
            for i in list(self.active):
                self.pool[i].active = False
            self.active.clear()
            self.gnbs = [GNB(0, 0.0, 0.0, 46.0, "gNB-0", 30.0, 28.0, 400.0, 64)]
            self._rebuild_hex()

    def run(self, running):
        while running[0]:
            t0 = time.time(); self.step()
            time.sleep(max(0, CFG["TICK_MS"]/1000.0 - (time.time()-t0)))

                                                               
                
                                                               
DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>5G NR Digital Twin — NS3 Engine v12</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;400;600;800&display=swap');
:root{
  --bg:#060b14;--bg2:#0c1424;--bg3:#111d30;--bg4:#0a1520;
  --border:#1a3a5c;--accent:#00e5ff;--accent2:#cc00ff;
  --green:#00ff9d;--red:#ff3d5a;--yellow:#ffcc00;--orange:#ff8c00;
  --pink:#ff69b4;--teal:#00ced1;
  --text:#c8dff0;--dim:#4a6880;
  --glow:0 0 16px rgba(0,229,255,0.35);
  --glow2:0 0 16px rgba(204,0,255,0.35);
}
*{box-sizing:border-box;margin:0;padding:0;}
html,body{background:var(--bg);color:var(--text);font-family:'Exo 2',sans-serif;height:100vh;overflow:hidden;}
header{display:flex;align-items:center;justify-content:space-between;padding:6px 20px;
  border-bottom:1px solid var(--border);background:var(--bg2);position:sticky;top:0;z-index:200;height:46px;}
.logo{font-size:1.1rem;font-weight:800;letter-spacing:2px;color:var(--accent);text-shadow:var(--glow);}
.logo span{color:var(--accent2);}
.hdr-pills{display:flex;gap:6px;align-items:center;flex-wrap:wrap;}
.pill{padding:2px 8px;border-radius:99px;font-size:0.62rem;font-weight:700;letter-spacing:1px;font-family:'Share Tech Mono',monospace;}
.pill.green{background:rgba(0,255,157,0.1);color:var(--green);border:1px solid var(--green);}
.pill.red{background:rgba(255,61,90,0.1);color:var(--red);border:1px solid var(--red);}
.pill.yellow{background:rgba(255,204,0,0.1);color:var(--yellow);border:1px solid var(--yellow);}
.pill.blue{background:rgba(0,229,255,0.1);color:var(--accent);border:1px solid var(--accent);}
.pill.purple{background:rgba(204,0,255,0.1);color:var(--accent2);border:1px solid var(--accent2);}
.pill.orange{background:rgba(255,140,0,0.1);color:var(--orange);border:1px solid var(--orange);}
.pill.teal{background:rgba(0,206,209,0.1);color:var(--teal);border:1px solid var(--teal);}
.layout{display:grid;grid-template-columns:280px 1fr 320px;height:calc(100vh - 46px);overflow:hidden;}
.panel{background:var(--bg2);border-right:1px solid var(--border);overflow-y:auto;display:flex;flex-direction:column;}
.panel-right{border-right:none;border-left:1px solid var(--border);}
.panel-section{border-bottom:1px solid var(--border);}
.sec-hdr{font-size:0.58rem;font-weight:700;letter-spacing:3px;text-transform:uppercase;
  color:var(--dim);padding:7px 14px 5px;background:var(--bg3);
  display:flex;align-items:center;justify-content:space-between;cursor:pointer;user-select:none;}
.sec-hdr:hover{color:var(--accent);}
.sec-body{padding:7px 12px;}
.kpi-grid{display:grid;grid-template-columns:1fr 1fr;gap:5px;padding:7px 10px;}
.kpi{background:var(--bg3);border:1px solid var(--border);border-radius:6px;padding:7px 10px;position:relative;overflow:hidden;}
.kpi::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;}
.kpi.twin::before{background:var(--accent);}
.kpi.real::before{background:var(--green);}
.kpi.delta::before{background:var(--accent2);}
.kpi.embb::before{background:var(--accent);}
.kpi.urllc::before{background:var(--yellow);}
.kpi.mmtc::before{background:var(--accent2);}
.kpi-label{font-size:0.52rem;letter-spacing:2px;color:var(--dim);text-transform:uppercase;}
.kpi-value{font-family:'Share Tech Mono',monospace;font-size:0.95rem;color:var(--accent);margin-top:1px;}
.kpi.real .kpi-value{color:var(--green);}
.kpi.delta .kpi-value{color:var(--accent2);}
.kpi.urllc .kpi-value{color:var(--yellow);}
.kpi.mmtc .kpi-value{color:var(--accent2);}
.kpi-unit{font-size:0.52rem;color:var(--dim);}
.cmp-table{width:100%;font-size:0.65rem;font-family:'Share Tech Mono',monospace;border-collapse:collapse;}
.cmp-table th{color:var(--dim);font-size:0.54rem;letter-spacing:1px;padding:4px 8px;text-align:left;border-bottom:1px solid var(--border);}
.cmp-table td{padding:3px 8px;border-bottom:1px solid rgba(26,58,92,0.3);}
.cmp-table tr:hover td{background:var(--bg3);}
.pos{color:var(--green);}.neg{color:var(--red);}.neu{color:var(--dim);}
.main{display:flex;flex-direction:column;overflow:hidden;}
.canvas-hdr{padding:6px 14px;border-bottom:1px solid var(--border);display:flex;align-items:center;gap:8px;flex-shrink:0;}
.canvas-hdr h2{font-size:0.72rem;letter-spacing:3px;color:var(--accent);text-transform:uppercase;}
.canvas-wrap{flex:1;position:relative;overflow:hidden;min-height:0;}
canvas{display:block;width:100%;height:100%;cursor:crosshair;}
.legend{display:flex;gap:8px;font-size:0.58rem;margin-left:auto;flex-wrap:wrap;}
.legend-item{display:flex;align-items:center;gap:4px;}
.dot{width:7px;height:7px;border-radius:50%;}
.ue-area{border-top:1px solid var(--border);height:200px;overflow:hidden;display:flex;flex-direction:column;flex-shrink:0;}
.ue-area-hdr{padding:5px 12px;border-bottom:1px solid var(--border);font-size:0.58rem;
  letter-spacing:3px;color:var(--dim);text-transform:uppercase;background:var(--bg3);
  display:flex;gap:8px;align-items:center;flex-shrink:0;}
.ue-scroll{overflow-y:auto;flex:1;}
.ue-table{width:100%;font-size:0.6rem;font-family:'Share Tech Mono',monospace;border-collapse:collapse;}
.ue-table th{color:var(--dim);font-size:0.52rem;letter-spacing:1px;padding:4px 6px;text-align:left;
  position:sticky;top:0;background:var(--bg2);border-bottom:1px solid var(--border);}
.ue-table td{padding:3px 6px;border-bottom:1px solid rgba(26,58,92,0.2);white-space:nowrap;}
.ue-table tr:hover td{background:var(--bg3);cursor:pointer;}
.ue-table tr.selected td{background:rgba(0,229,255,0.08);}
/* Slice colors — DISTINCT: eMBB=blue, URLLC=yellow, mMTC=magenta/purple */
.slice-embb{color:#0000ff;}
.slice-urllc{color:#ffcc00;}
.slice-mmtc{color:#cc00ff;}
.ue-ho{color:var(--red);}
.ctrl-section{padding:7px 12px 9px;border-bottom:1px solid var(--border);}
.ctrl-section h4{font-size:0.56rem;letter-spacing:2px;color:var(--dim);text-transform:uppercase;margin-bottom:7px;
  display:flex;justify-content:space-between;align-items:center;}
.ctrl-row{display:flex;gap:5px;margin-bottom:5px;align-items:center;}
.ctrl-row label{font-size:0.58rem;color:var(--dim);min-width:64px;flex-shrink:0;}
input[type=range]{flex:1;-webkit-appearance:none;height:3px;background:var(--border);border-radius:2px;outline:none;}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:11px;height:11px;border-radius:50%;background:var(--accent);cursor:pointer;}
input[type=number],input[type=text],select{
  background:var(--bg3);border:1px solid var(--border);color:var(--text);
  border-radius:5px;padding:3px 6px;font-size:0.68rem;font-family:'Exo 2',sans-serif;width:100%;}
input:focus,select:focus{outline:none;border-color:var(--accent);box-shadow:var(--glow);}
.inp-sm{width:56px !important;}.inp-md{width:72px !important;}
button{background:transparent;border:1px solid var(--accent);color:var(--accent);border-radius:5px;
  padding:4px 9px;font-size:0.65rem;font-family:'Exo 2',sans-serif;font-weight:600;
  letter-spacing:1px;cursor:pointer;transition:all 0.2s;white-space:nowrap;}
button:hover{background:rgba(0,229,255,0.1);box-shadow:var(--glow);}
button.danger{border-color:var(--red);color:var(--red);}
button.danger:hover{background:rgba(255,61,90,0.1);}
button.purple{border-color:var(--accent2);color:var(--accent2);}
button.purple:hover{background:rgba(204,0,255,0.1);}
button.green-btn{border-color:var(--green);color:var(--green);}
button.green-btn:hover{background:rgba(0,255,157,0.1);}
button.orange-btn{border-color:var(--orange);color:var(--orange);}
button.orange-btn:hover{background:rgba(255,140,0,0.1);}
button.full{width:100%;margin-top:3px;}
button.sm{padding:2px 6px;font-size:0.58rem;}
.range-val{font-family:'Share Tech Mono',monospace;font-size:0.62rem;min-width:42px;text-align:right;color:var(--accent);}
/* gNB cards — each gets a colored left border matching its canvas color */
.gnb-card{background:var(--bg3);border:1px solid var(--border);border-radius:6px;margin:5px 10px;padding:7px 10px;}
.gnb-card.selected{border-color:var(--accent);box-shadow:var(--glow);}
.gnb-card-hdr{display:flex;align-items:center;gap:5px;margin-bottom:5px;}
.gnb-dot{width:8px;height:8px;border-radius:50%;}
.gnb-name{font-size:0.72rem;font-weight:700;color:var(--accent);flex:1;}
.gnb-params{display:grid;grid-template-columns:1fr 1fr;gap:2px 8px;font-size:0.58rem;font-family:'Share Tech Mono',monospace;}
.gnb-param-label{color:var(--dim);}.gnb-param-val{color:var(--text);}
/* UE-per-gNB list */
.gnb-ue-summary{margin-top:5px;padding-top:5px;border-top:1px solid var(--border);}
.gnb-ue-summary-hdr{font-size:0.54rem;letter-spacing:1px;color:var(--dim);text-transform:uppercase;margin-bottom:3px;}
.gnb-ue-ids{font-family:'Share Tech Mono',monospace;font-size:0.56rem;color:var(--text);word-break:break-all;line-height:1.6;}
.gnb-ue-count-badge{display:inline-block;padding:1px 5px;border-radius:99px;font-size:0.5rem;
  font-weight:700;margin-left:5px;}
.real-row{display:flex;justify-content:space-between;font-size:0.62rem;font-family:'Share Tech Mono',monospace;padding:2px 0;}
.real-label{color:var(--dim);}.real-val{color:var(--green);}
.weather-btns{display:flex;gap:4px;flex-wrap:wrap;}
.weather-btn{padding:3px 7px;border-radius:4px;font-size:0.58rem;font-family:'Exo 2',sans-serif;
  font-weight:600;cursor:pointer;border:1px solid var(--border);background:transparent;color:var(--dim);transition:all 0.2s;}
.weather-btn:hover,.weather-btn.active{border-color:var(--accent);color:var(--accent);background:rgba(0,229,255,0.08);}
#ue-detail{position:absolute;bottom:210px;left:50%;transform:translateX(-50%);
  background:var(--bg3);border:1px solid var(--accent);border-radius:8px;
  padding:10px 14px;font-size:0.63rem;font-family:'Share Tech Mono',monospace;
  display:none;z-index:100;min-width:270px;box-shadow:var(--glow);}
#ue-detail h5{color:var(--accent);margin-bottom:5px;font-size:0.68rem;letter-spacing:1px;}
#ue-detail .det-grid{display:grid;grid-template-columns:1fr 1fr;gap:2px 14px;}
.close-det{float:right;cursor:pointer;color:var(--red);}
.tab-bar{display:flex;border-bottom:1px solid var(--border);background:var(--bg3);}
.tab{padding:6px 8px;font-size:0.58rem;letter-spacing:1px;text-transform:uppercase;
  cursor:pointer;color:var(--dim);border-bottom:2px solid transparent;transition:all 0.2s;}
.tab:hover{color:var(--text);}
.tab.active{color:var(--accent);border-bottom-color:var(--accent);}
.tab-content{display:none;flex:1;overflow-y:auto;}
.tab-content.active{display:flex;flex-direction:column;}
#toast{position:fixed;bottom:18px;right:18px;background:var(--bg3);border:1px solid var(--accent);
  color:var(--accent);padding:7px 14px;border-radius:7px;font-size:0.72rem;
  font-family:'Share Tech Mono',monospace;opacity:0;transition:opacity 0.3s;pointer-events:none;z-index:999;}
#toast.show{opacity:1;}
::-webkit-scrollbar{width:4px;height:4px;}
::-webkit-scrollbar-track{background:var(--bg);}
::-webkit-scrollbar-thumb{background:var(--border);border-radius:2px;}
.collapsible .sec-body{display:block;}
.collapsible.collapsed .sec-body{display:none;}
.params-grid{display:grid;grid-template-columns:1fr 1fr;gap:4px 6px;font-size:0.6rem;font-family:'Share Tech Mono',monospace;}
.p-label{color:var(--dim);padding-top:2px;}
.slice-kpi-table{width:100%;font-size:0.62rem;font-family:'Share Tech Mono',monospace;border-collapse:collapse;margin:4px 0;}
.slice-kpi-table th{color:var(--dim);font-size:0.52rem;letter-spacing:1px;padding:3px 8px;text-align:left;border-bottom:1px solid var(--border);}
.slice-kpi-table td{padding:3px 8px;}
.slice-kpi-table .embb-row td:first-child{color:#00e5ff;}
.slice-kpi-table .urllc-row td:first-child{color:#ffcc00;}
.slice-kpi-table .mmtc-row td:first-child{color:#cc00ff;}
.warn-badge{font-size:0.5rem;color:var(--red);margin-left:3px;}
.sample-ue-grid{display:grid;grid-template-columns:1fr;gap:5px;padding:6px 8px;}
.sample-ue{background:var(--bg3);border:1px solid var(--border);border-radius:6px;padding:5px 9px;}
.sample-ue-hdr{display:flex;align-items:center;gap:5px;margin-bottom:3px;}
.sample-ue-id{font-size:0.62rem;font-weight:700;font-family:'Share Tech Mono',monospace;}
.sample-ue-metrics{display:grid;grid-template-columns:1fr 1fr 1fr;gap:1px 6px;font-size:0.56rem;font-family:'Share Tech Mono',monospace;}
.sm-label{color:var(--dim);}.sm-val{color:var(--text);}
/* Map toggle */
#map-toggle{font-size:0.6rem;padding:2px 7px;border-radius:4px;cursor:pointer;
  border:1px solid var(--border);background:transparent;color:var(--dim);}
#map-toggle.on{border-color:var(--teal);color:var(--teal);}
</style>
</head>
<body>
<header>
  <div class="logo">5G<span>TWIN</span> <span style="font-size:0.62rem;color:var(--dim);letter-spacing:1px">DIGITAL TWIN v11 — OSM MAP</span></div>
  <div class="hdr-pills">
    <span id="sync-status" class="pill yellow">CONNECTING…</span>
    <span id="twin-ues" class="pill blue">TWIN: 0 UEs</span>
    <span id="real-ues" class="pill green">REAL: 0 UEs</span>
    <span id="gnb-badge" class="pill teal">gNBs: 1</span>
    <span id="cap-badge" class="pill orange">CAP: 250</span>
    <span id="weather-badge" class="pill orange">☀ NORMAL</span>
    <span id="sched-badge" class="pill purple">PF</span>
    <button id="map-toggle" class="on" onclick="toggleMap()">🗺 MAP ON</button>
    <span id="clock" style="font-family:'Share Tech Mono',monospace;font-size:0.68rem;color:var(--dim)"></span>
  </div>
</header>

<div class="layout">
<!-- LEFT PANEL -->
<div class="panel">
  <div class="panel-section collapsible" id="sec-kpi">
    <div class="sec-hdr" onclick="toggleSection('sec-kpi')">▾ TWIN vs REAL — KPIs</div>
    <div class="sec-body">
      <div class="kpi-grid" id="kpi-grid"></div>
    </div>
  </div>
  <div class="panel-section collapsible" id="sec-slices">
    <div class="sec-hdr" onclick="toggleSection('sec-slices')">▾ SLICE AVG METRICS</div>
    <div class="sec-body" style="padding:4px 8px">
      <table class="slice-kpi-table">
        <thead><tr><th>SLICE</th><th>UEs</th><th>Avg Tput</th><th>Avg Lat</th></tr></thead>
        <tbody id="slice-tbody"></tbody>
      </table>
    </div>
  </div>
  <div class="panel-section collapsible" id="sec-delta">
    <div class="sec-hdr" onclick="toggleSection('sec-delta')">▾ DELTA COMPARISON</div>
    <div class="sec-body" style="padding:0 0 5px">
      <table class="cmp-table">
        <thead><tr><th>METRIC</th><th>TWIN</th><th>REAL</th><th>Δ</th></tr></thead>
        <tbody id="cmp-tbody"></tbody>
      </table>
    </div>
  </div>
  <div class="panel-section collapsible" id="sec-gnbs">
    <div class="sec-hdr" onclick="toggleSection('sec-gnbs')">▾ ACTIVE gNBs + CONNECTED UEs</div>
    <div class="sec-body" style="padding:3px 0" id="gnb-list"></div>
  </div>
  <div class="panel-section collapsible" id="sec-real">
    <div class="sec-hdr" onclick="toggleSection('sec-real')">▾ REAL NETWORK LIVE</div>
    <div class="sec-body" style="padding:3px 0" id="real-panel"></div>
  </div>
  <div class="panel-section collapsible" id="sec-sys">
    <div class="sec-hdr" onclick="toggleSection('sec-sys')">▾ SYSTEM CONFIG (LIVE)</div>
    <div class="sec-body" id="sys-cfg"></div>
  </div>
</div>

<!-- CENTRE -->
<div class="main">
  <div class="canvas-hdr">
    <h2>CELL TOPOLOGY</h2>
    <span style="font-size:0.58rem;color:var(--dim)" id="canvas-hint">Left-click: place UE · Right-click: place gNB</span>
    <div class="legend">
      <div class="legend-item"><div class="dot" style="background:#00e5ff"></div><span>eMBB</span></div>
      <div class="legend-item"><div class="dot" style="background:#ffcc00"></div><span>URLLC</span></div>
      <div class="legend-item"><div class="dot" style="background:#cc00ff"></div><span>mMTC</span></div>
      <div class="legend-item"><div class="dot" style="background:#ff3d5a"></div><span>HO</span></div>
      <div class="legend-item"><div class="dot" style="background:#00ff9d;box-shadow:0 0 4px #00ff9d"></div><span>gNB</span></div>
    </div>
  </div>
  <div class="canvas-wrap">
    <canvas id="topo"></canvas>
    <div id="ue-detail">
      <h5>UE DETAIL <span class="close-det" onclick="closeDetail()">✕</span></h5>
      <div class="det-grid" id="det-grid"></div>
    </div>
  </div>
  <div class="ue-area">
    <div class="ue-area-hdr">
      <span>ACTIVE UEs (TWIN)</span>
      <span id="ue-count-badge" class="pill blue" style="font-size:0.52rem">0</span>
      <span style="margin-left:auto;font-size:0.56rem;color:var(--dim)">Click row for detail</span>
    </div>
    <div class="ue-scroll">
      <table class="ue-table">
        <thead><tr>
          <th>ID</th><th>SLICE</th><th>NAS</th><th>SINR</th><th>CQI</th>
          <th>MOD</th><th>LYR</th><th>TPUT</th><th>LAT</th><th>BLER</th>
          <th>RSRP</th><th>RSRQ</th><th>DOPP</th><th>SPD</th><th>RB</th>
          <th>gNB</th><th>HO</th><th>IP</th><th>×</th>
        </tr></thead>
        <tbody id="ue-tbody"></tbody>
      </table>
    </div>
  </div>
</div>

<!-- RIGHT CONTROL PANEL -->
<div class="panel panel-right">
  <div class="tab-bar">
    <div class="tab active" onclick="switchTab('tab-gnb')">gNB</div>
    <div class="tab" onclick="switchTab('tab-ue')">UE</div>
    <div class="tab" onclick="switchTab('tab-sys')">SYS</div>
    <div class="tab" onclick="switchTab('tab-sample')">SAMPLE</div>
    <div class="tab" onclick="switchTab('tab-pcap');loadPcapStats()">PCAP</div>
  </div>

  <!-- gNB TAB -->
  <div class="tab-content active" id="tab-gnb">
    <div class="ctrl-section">
      <h4>Add New gNB</h4>
      <div class="ctrl-row"><label>Label</label><input type="text" id="gnb-label" value="gNB-Twin" style="width:100%"></div>
      <div class="ctrl-row"><label>X (m)</label><input type="number" id="gnb-x" value="300" class="inp-md">
        <label style="min-width:18px">Y</label><input type="number" id="gnb-y" value="0" class="inp-md"></div>
      <div class="ctrl-row"><label>Tx Power</label>
        <input type="range" id="gnb-tx-s" min="20" max="60" step="0.5" value="46" oninput="sv('gnb-tx-v',this.value,'dBm')">
        <span class="range-val" id="gnb-tx-v">46dBm</span></div>
      <div class="ctrl-row"><label>Height (m)</label>
        <input type="range" id="gnb-h-s" min="5" max="200" step="1" value="30" oninput="sv('gnb-h-v',this.value,'m')">
        <span class="range-val" id="gnb-h-v">30m</span></div>
      <div class="ctrl-row"><label>Freq (GHz)</label>
        <input type="range" id="gnb-fq-s" min="0.7" max="60" step="0.1" value="28" oninput="sv('gnb-fq-v',this.value,'GHz');updateGnbRbHint()">
        <span class="range-val" id="gnb-fq-v">28GHz</span></div>
      <div class="ctrl-row"><label>BW (MHz)</label>
        <input type="range" id="gnb-bw-s" min="5" max="400" step="5" value="400" oninput="sv('gnb-bw-v',this.value,'MHz');updateGnbRbHint()">
        <span class="range-val" id="gnb-bw-v">400MHz</span></div>
      <div style="font-size:0.54rem;color:var(--dim);margin:2px 0 4px" id="gnb-rb-hint">→ ~270 RBs (30kHz SCS)</div>
      <div class="ctrl-row"><label>TX Ant (MIMO)</label>
        <select id="gnb-ant">
          <option value="4">4 (2×2)</option><option value="8">8 (4×2)</option>
          <option value="16">16 (4×4)</option><option value="32">32 (8×4)</option>
          <option value="64" selected>64 (massive)</option><option value="128">128 (ultra)</option>
        </select></div>
      <div class="ctrl-row"><label>Max Layers</label>
        <select id="gnb-layers">
          <option value="1">1</option><option value="2">2</option>
          <option value="4" selected>4</option><option value="8">8</option></select></div>
      <div style="font-size:0.56rem;color:var(--dim);margin:3px 0 5px">RB allocation (this gNB):</div>
      <div class="ctrl-row"><label>eMBB RBs</label>
        <input type="range" id="gnb-rbe" min="0" max="270" step="1" value="216" oninput="sv('gnb-rbe-v',this.value,'')">
        <span class="range-val" id="gnb-rbe-v">216</span></div>
      <div class="ctrl-row"><label>URLLC RBs</label>
        <input type="range" id="gnb-rbu" min="0" max="270" step="1" value="27" oninput="sv('gnb-rbu-v',this.value,'')">
        <span class="range-val" id="gnb-rbu-v">27</span></div>
      <div class="ctrl-row"><label>mMTC RBs</label>
        <input type="range" id="gnb-rbm" min="0" max="270" step="1" value="27" oninput="sv('gnb-rbm-v',this.value,'')">
        <span class="range-val" id="gnb-rbm-v">27</span></div>
      <button class="full green-btn" onclick="addGnb()">＋ ADD gNB</button>
    </div>
    <div class="ctrl-section">
      <h4>Edit Selected gNB <span id="sel-gnb-lbl" style="color:var(--accent);font-size:0.68rem"></span></h4>
      <div style="font-size:0.6rem;color:var(--dim);margin-bottom:5px">Click a gNB card on left panel</div>
      <div id="gnb-edit-form" style="display:none">
        <div class="ctrl-row"><label>Label</label><input type="text" id="edit-gnb-label"></div>
        <div class="ctrl-row"><label>X (m)</label><input type="number" id="edit-gnb-x" class="inp-md">
          <label style="min-width:18px">Y</label><input type="number" id="edit-gnb-y" class="inp-md"></div>
        <div class="ctrl-row"><label>Tx Power</label>
          <input type="range" id="edit-gnb-tx" min="20" max="60" step="0.5" value="46" oninput="sv('edit-gnb-tx-v',this.value,'dBm')">
          <span class="range-val" id="edit-gnb-tx-v">46dBm</span></div>
        <div class="ctrl-row"><label>Height (m)</label>
          <input type="range" id="edit-gnb-h" min="5" max="200" step="1" value="30" oninput="sv('edit-gnb-h-v',this.value,'m')">
          <span class="range-val" id="edit-gnb-h-v">30m</span></div>
        <div class="ctrl-row"><label>Freq (GHz)</label>
          <input type="range" id="edit-gnb-fq" min="0.7" max="60" step="0.1" value="28" oninput="sv('edit-gnb-fq-v',this.value,'GHz')">
          <span class="range-val" id="edit-gnb-fq-v">28GHz</span></div>
        <div class="ctrl-row"><label>BW (MHz)</label>
          <input type="range" id="edit-gnb-bw" min="5" max="400" step="5" value="400" oninput="sv('edit-gnb-bw-v',this.value,'MHz')">
          <span class="range-val" id="edit-gnb-bw-v">400MHz</span></div>
        <div class="ctrl-row"><label>TX Ant</label>
          <select id="edit-gnb-ant">
            <option value="4">4</option><option value="8">8</option>
            <option value="16">16</option><option value="32">32</option>
            <option value="64">64</option><option value="128">128</option>
          </select></div>
        <div class="ctrl-row"><label>Max Lyr</label>
          <select id="edit-gnb-layers">
            <option value="1">1</option><option value="2">2</option>
            <option value="4">4</option><option value="8">8</option>
          </select></div>
        <div style="display:flex;gap:4px;margin-top:3px">
          <button style="flex:1" onclick="applyGnbEdit()">✓ APPLY</button>
          <button class="danger sm" onclick="deleteGnb()">🗑 DEL</button>
        </div>
      </div>
    </div>
  </div>

  <!-- UE TAB -->
  <div class="tab-content" id="tab-ue">
    <div class="ctrl-section">
      <h4>Add UEs</h4>
      <div class="ctrl-row"><label>Count</label><input type="number" id="ue-count" value="10" class="inp-sm">
        <label style="min-width:28px">Slice</label>
        <select id="ue-slice">
          <option value="embb">eMBB</option>
          <option value="urllc">URLLC</option>
          <option value="mmtc">mMTC</option>
        </select></div>
      <div class="ctrl-row"><label>X (m)</label><input type="number" id="ue-x" placeholder="random" class="inp-md">
        <label style="min-width:18px">Y</label><input type="number" id="ue-y" placeholder="random" class="inp-md"></div>
      <div class="ctrl-row"><label>Speed (m/s)</label>
        <input type="range" id="ue-speed-s" min="0" max="120" step="0.5" value="2" oninput="sv('ue-speed-v',this.value,'m/s')">
        <span class="range-val" id="ue-speed-v">2m/s</span></div>
      <button class="full" onclick="addUes()">＋ ADD UEs</button>
    </div>
    <div class="ctrl-section">
      <h4>Move / Remove UE</h4>
      <div class="ctrl-row"><label>UE ID</label><input type="number" id="move-id" value="0" class="inp-sm"></div>
      <div class="ctrl-row"><label>X (m)</label><input type="number" id="move-x" value="100" class="inp-md">
        <label style="min-width:18px">Y</label><input type="number" id="move-y" value="50" class="inp-md"></div>
      <div style="display:flex;gap:4px;margin-top:3px">
        <button class="purple" style="flex:1" onclick="moveUe()">↗ MOVE</button>
        <button class="danger sm" onclick="removeUeById()">🗑 RM</button>
      </div>
    </div>
    <div class="ctrl-section">
      <h4>Bulk Operations</h4>
      <div class="ctrl-row"><label>Remove N</label><input type="number" id="bulk-rm" value="10" class="inp-sm">
        <button class="danger sm" style="flex:1" onclick="bulkRemove()">🗑 REMOVE</button></div>
    </div>
    <div class="ctrl-section">
      <h4>RB Allocation (Global / All gNBs)</h4>
      <div style="font-size:0.56rem;color:var(--dim);margin-bottom:4px">Proportionally applied to all gNBs</div>
      <div class="ctrl-row"><label>eMBB RBs</label>
        <input type="range" id="r-embb" min="0" max="270" step="1" value="216" oninput="sv('v-embb',this.value,'')">
        <span class="range-val" id="v-embb">216</span></div>
      <div class="ctrl-row"><label>URLLC RBs</label>
        <input type="range" id="r-urllc" min="0" max="270" step="1" value="27" oninput="sv('v-urllc',this.value,'')">
        <span class="range-val" id="v-urllc">27</span></div>
      <div class="ctrl-row"><label>mMTC RBs</label>
        <input type="range" id="r-mmtc" min="0" max="270" step="1" value="27" oninput="sv('v-mmtc',this.value,'')">
        <span class="range-val" id="v-mmtc">27</span></div>
      <button class="full" onclick="applyRbAlloc()">📊 APPLY RB (All gNBs)</button>
      <div class="ctrl-row" style="margin-top:6px"><label>Scheduler</label>
        <select id="r-sched" onchange="applyScheduler()">
          <option value="PF">PF — Proportional Fair</option>
          <option value="RR">RR — Round Robin</option>
          <option value="MAXCQI">MaxCQI — Greedy</option>
        </select></div>
    </div>
  </div>

  <!-- SYS TAB -->
  <div class="tab-content" id="tab-sys">
    <div class="ctrl-section">
      <h4>Weather Conditions</h4>
      <div class="weather-btns">
        <button class="weather-btn active" id="w-NORMAL" onclick="setWeather('NORMAL')">☀ Normal</button>
        <button class="weather-btn" id="w-RAINY"  onclick="setWeather('RAINY')">🌧 Rainy</button>
        <button class="weather-btn" id="w-WINDY"  onclick="setWeather('WINDY')">💨 Windy</button>
        <button class="weather-btn" id="w-FOGGY"  onclick="setWeather('FOGGY')">🌫 Foggy</button>
      </div>
      <div id="weather-desc" style="margin-top:5px;font-size:0.58rem;color:var(--dim)">No propagation impairments</div>
    </div>
    <div class="ctrl-section">
      <h4>Cell Geometry</h4>
      <div class="ctrl-row"><label>Cell Radius</label>
        <input type="range" id="r-cr" min="100" max="2000" step="50" value="500" oninput="sv('v-cr',this.value,'m')">
        <span class="range-val" id="v-cr">500m</span></div>
      <div class="ctrl-row"><label>ISD (m)</label>
        <input type="range" id="r-isd" min="200" max="5000" step="100" value="1000" oninput="sv('v-isd',this.value,'m')">
        <span class="range-val" id="v-isd">1000m</span></div>
      <button class="full" onclick="applyCellGeom()">📐 APPLY GEOMETRY</button>
    </div>
    <div class="ctrl-section">
      <h4>System Config (Live)</h4>
      <div id="sys-cfg-tab"></div>
    </div>
    <div class="ctrl-section">
      <button class="full danger" style="margin-top:3px" onclick="resetTwin()">↺ RESET TWIN</button>
    </div>
  </div>

  <!-- SAMPLE UEs TAB -->
  <div class="tab-content" id="tab-sample">
    <div class="ctrl-section">
      <h4>5 Random UEs per Slice — Live Metrics</h4>
      <div style="font-size:0.56rem;color:var(--dim);margin-bottom:5px">Per-UE PHY state refreshed every tick</div>
    </div>
    <div style="overflow-y:auto;flex:1">
      <div style="padding:4px 8px;font-size:0.58rem;letter-spacing:2px;color:#00e5ff;text-transform:uppercase">eMBB Samples</div>
      <div class="sample-ue-grid" id="sample-embb"></div>
      <div style="padding:4px 8px;font-size:0.58rem;letter-spacing:2px;color:#ffcc00;text-transform:uppercase;border-top:1px solid var(--border)">URLLC Samples</div>
      <div class="sample-ue-grid" id="sample-urllc"></div>
      <div style="padding:4px 8px;font-size:0.58rem;letter-spacing:2px;color:#cc00ff;text-transform:uppercase;border-top:1px solid var(--border)">mMTC Samples</div>
      <div class="sample-ue-grid" id="sample-mmtc"></div>
    </div>
  </div>

  <!-- PCAP TAB -->
  <div class="tab-content" id="tab-pcap">
    <div class="ctrl-section">
      <h4>🦈 PCAP Packet Capture</h4>
      <div style="font-size:0.56rem;color:var(--dim);margin-bottom:6px">NS3 interface capture. Open .pcap files in Wireshark.</div>
      <div id="pcap-stats-grid" style="display:grid;gap:5px"></div>
      <button onclick="loadPcapStats()" style="margin-top:8px;width:100%;padding:5px;background:var(--border);border:1px solid var(--accent);color:var(--fg);border-radius:4px;cursor:pointer;font-size:0.65rem">↻ Refresh Stats</button>
    </div>
    <div class="ctrl-section" style="flex:1;overflow-y:auto">
      <h4>Download .pcap</h4>
      <div style="font-size:0.6rem;color:var(--dim);margin-bottom:6px">Click interface to download live packet capture</div>
      <div id="pcap-dl-list" style="display:flex;flex-direction:column;gap:5px"></div>
    </div>
    <div class="ctrl-section">
      <h4>Interface Legend</h4>
      <div style="font-size:0.58rem;color:var(--dim);line-height:1.7">
        <b style="color:var(--accent)">uu_radio</b> — PHY/MAC/RLC/PDCP/RRC (DLT_USER0)<br>
        <b style="color:var(--yellow)">n2_ngap</b> — SCTP/NGAP gNB↔AMF (DLT_EN10MB)<br>
        <b style="color:#cc00ff">n3_gtpu</b> — GTP-U/UDP gNB↔UPF (DLT_EN10MB)<br>
        <b style="color:#00e5ff">f1_du_cu</b> — UDP/F1AP DU↔CU (DLT_EN10MB)<br>
        <b style="color:#ff9800">sba_http2</b> — HTTP/2 SBA mesh (DLT_EN10MB)<br>
        <b style="color:#e91e63">coap_mmtc</b> — CoAP mMTC sensors (DLT_EN10MB)<br>
        <b style="color:#4caf50">icmp_ctrl</b> — ICMP/ARP ctrl (DLT_EN10MB)
      </div>
    </div>
  </div>
</div>
</div>

<div id="toast"></div>

<script>
/* ══════════════════════════════════════════
   OSM MAP DATA — embedded from map.osm
   ══════════════════════════════════════════ */
const OSM_DATA = {"b":{"minlat":12.931518,"minlon":77.528447,"maxlat":12.941065,"maxlon":77.543961},"r":[{"t":"r","c":[[12.940843,77.542729],[12.940445,77.542695],[12.940075,77.542663],[12.939656,77.542627],[12.939233,77.54259],[12.937702,77.542458],[12.937583,77.542447]]},{"t":"r","c":[[12.937787,77.539914],[12.937789,77.540058],[12.937709,77.540366],[12.937644,77.540615],[12.937638,77.540638],[12.93763,77.540667],[12.937577,77.54087],[12.937504,77.541143],[12.93743,77.541411],[12.93733,77.541773],[12.937127,77.542419],[12.936927,77.543056],[12.936616,77.544049]]},{"t":"r","c":[[12.934248,77.541808],[12.935482,77.54217],[12.935674,77.542229],[12.935929,77.542302],[12.936282,77.542408]]},{"t":"r","c":[[12.93402,77.543799],[12.934929,77.544101],[12.935281,77.54424]]},{"t":"r","c":[[12.935281,77.54424],[12.935477,77.543643],[12.935805,77.542686],[12.935929,77.542302],[12.936046,77.541811],[12.936076,77.541685],[12.936076,77.541462],[12.936041,77.540611],[12.936012,77.540288]]},{"t":"r","c":[[12.935566,77.540169],[12.935609,77.54053],[12.935647,77.54093],[12.93567,77.541353],[12.935641,77.541613],[12.935622,77.541783],[12.935482,77.54217]]},{"t":"r","c":[[12.936076,77.541462],[12.93567,77.541353],[12.934449,77.540995]]},{"t":"r","c":[[12.933301,77.544581],[12.933507,77.544073],[12.933774,77.543276],[12.934144,77.542167],[12.934248,77.541808],[12.934348,77.541394],[12.934449,77.540995],[12.934483,77.540547],[12.93451,77.540189],[12.93448,77.53988]]},{"t":"r","c":[[12.93388,77.544197],[12.93402,77.543799],[12.934138,77.543394],[12.934502,77.542271]]},{"t":"r","c":[[12.935805,77.542686],[12.935413,77.542574],[12.935029,77.542435],[12.934502,77.542271],[12.934144,77.542167]]},{"t":"r","c":[[12.934014,77.546303],[12.934231,77.545927],[12.93449,77.545479],[12.93462,77.545036],[12.93479,77.544501],[12.934929,77.544101],[12.935043,77.543703],[12.935413,77.542574]]},{"t":"r","c":[[12.936282,77.542408],[12.936481,77.541724],[12.936489,77.541513],[12.936505,77.541131],[12.936518,77.54079]]},{"t":"r","c":[[12.936041,77.540611],[12.936518,77.54079],[12.937504,77.541143],[12.937941,77.541246]]},{"t":"r","c":[[12.937583,77.542447],[12.937845,77.541568],[12.937941,77.541246]]},{"t":"r","c":[[12.936489,77.541513],[12.936896,77.54164],[12.93733,77.541773]]},{"t":"r","c":[[12.936896,77.54164],[12.936527,77.542913],[12.936518,77.54294],[12.936191,77.54393]]},{"t":"r","c":[[12.93451,77.540189],[12.93515,77.54043],[12.935609,77.54053]]},{"t":"r","c":[[12.935647,77.54093],[12.934483,77.540547]]},{"t":"r","c":[[12.935622,77.541783],[12.934348,77.541394]]},{"t":"r","c":[[12.934138,77.543394],[12.934662,77.543573],[12.935043,77.543703]]},{"t":"r","c":[[12.934662,77.543573],[12.935029,77.542435]]},{"t":"r","c":[[12.93243,77.544138],[12.932633,77.543521],[12.933149,77.54196],[12.933476,77.540967],[12.933493,77.540651]]},{"t":"r","c":[[12.932723,77.541836],[12.932625,77.542075],[12.932465,77.542568],[12.932186,77.543465],[12.931879,77.543347],[12.931858,77.543341],[12.931786,77.54332],[12.931144,77.54293]]},{"t":"r","c":[[12.938574,77.536777],[12.938455,77.535714],[12.938349,77.534962],[12.938275,77.53421]]},{"t":"r","c":[[12.938047,77.534221],[12.938275,77.53421]]},{"t":"r","c":[[12.938381,77.53681],[12.938227,77.53574],[12.938047,77.534221]]},{"t":"r","c":[[12.937867,77.536549],[12.937646,77.536524],[12.937578,77.536517],[12.93742,77.536523],[12.937352,77.536525],[12.937038,77.536487],[12.936647,77.536476],[12.936429,77.536477],[12.936328,77.536475],[12.93624,77.536479],[12.935946,77.536481],[12.93554,77.536496],[12.935458,77.536469]]},{"t":"r","c":[[12.939363,77.535601],[12.939163,77.53564],[12.938893,77.535663],[12.938693,77.535681],[12.938455,77.535714],[12.938227,77.53574],[12.937998,77.535762],[12.93778,77.535798],[12.937609,77.535819],[12.937451,77.535841],[12.93708,77.534221]]},{"t":"r","c":[[12.939002,77.53398],[12.938988,77.534231],[12.938973,77.534445],[12.939163,77.53564],[12.939251,77.53618],[12.939327,77.536717],[12.939328,77.536973]]},{"t":"r","c":[[12.938988,77.534231],[12.938753,77.534232],[12.938687,77.534232],[12.938515,77.534222],[12.938275,77.53421]]},{"t":"r","c":[[12.938813,77.536755],[12.938693,77.535681],[12.938515,77.534222]]},{"t":"r","c":[[12.939789,77.536014],[12.940054,77.535982],[12.940362,77.535959],[12.940656,77.535928],[12.940566,77.534949],[12.940474,77.534015]]},{"t":"r","c":[[12.940054,77.535982],[12.939964,77.535003],[12.939876,77.533961]]},{"t":"r","c":[[12.939789,77.536014],[12.939714,77.535612],[12.939686,77.535352],[12.939662,77.535123],[12.93964,77.534921],[12.939607,77.534616],[12.939578,77.534337],[12.939544,77.533974]]},{"t":"r","c":[[12.939523,77.536689],[12.939406,77.535909],[12.939368,77.535656],[12.939363,77.535601],[12.939327,77.535383],[12.939285,77.535198],[12.939235,77.534975],[12.939191,77.534824],[12.939146,77.534688],[12.939129,77.534392],[12.939114,77.533983]]},{"t":"s","c":[[12.943539,77.538685],[12.943519,77.53819],[12.943508,77.537916],[12.943498,77.537683],[12.943476,77.53715],[12.943433,77.536871],[12.943411,77.536784],[12.943321,77.536691],[12.94306,77.536583],[12.942813,77.536395],[12.942635,77.536173],[12.942367,77.535944],[12.941972,77.535789],[12.941184,77.535582],[12.941048,77.535528],[12.940984,77.535468],[12.940946,77.534957],[12.940871,77.534387],[12.940789,77.534251],[12.94068,77.534133],[12.940596,77.534085],[12.940474,77.534015],[12.940159,77.533969],[12.939876,77.533961],[12.939544,77.533974],[12.939114,77.533983],[12.939002,77.53398],[12.938708,77.533739],[12.938495,77.533519]]},{"t":"t","c":[[12.939976,77.532531],[12.939589,77.532769]]},{"t":"r","c":[[12.939328,77.536973],[12.93932,77.536978],[12.938575,77.537072],[12.938017,77.53726],[12.937674,77.537365],[12.937467,77.537416]]},{"t":"r","c":[[12.939523,77.536689],[12.939327,77.536717],[12.939021,77.53674],[12.938813,77.536755],[12.938574,77.536777],[12.938381,77.53681],[12.93821,77.536828],[12.938141,77.536835],[12.937941,77.536859]]},{"t":"t","c":[[12.939297,77.531761],[12.939263,77.53151],[12.939189,77.531071],[12.939121,77.530669],[12.939049,77.530247],[12.938973,77.529841],[12.938905,77.529435],[12.938841,77.529077],[12.938776,77.528747],[12.93876,77.528432],[12.938748,77.527896],[12.938769,77.527727]]},{"t":"r","c":[[12.938776,77.528747],[12.938471,77.528805],[12.938122,77.528679],[12.937867,77.528597],[12.937839,77.528587],[12.937596,77.528502],[12.937435,77.528445],[12.937078,77.52832],[12.937049,77.52831],[12.936777,77.528214],[12.936698,77.528186],[12.936479,77.528109],[12.936249,77.528034],[12.936191,77.528016],[12.936165,77.528007],[12.935542,77.527804],[12.935177,77.527686]]},{"t":"m","c":[[12.928218,77.531759],[12.930619,77.531583],[12.932341,77.531449],[12.932662,77.531422],[12.932959,77.531398],[12.933252,77.531374],[12.933341,77.531366]]},{"t":"r","c":[[12.939233,77.54259],[12.939253,77.542199],[12.939278,77.541778],[12.9393,77.541388],[12.939435,77.539023]]},{"t":"r","c":[[12.939695,77.541824],[12.939278,77.541778],[12.938068,77.541642]]},{"t":"r","c":[[12.938881,77.541366],[12.938938,77.540582],[12.93897,77.540133],[12.939017,77.539565],[12.939041,77.539112]]},{"t":"t","c":[[12.940998,77.541744],[12.941605,77.541717],[12.942985,77.541656],[12.943385,77.541638],[12.943795,77.54162],[12.944141,77.541596],[12.944297,77.541577],[12.9444,77.541557],[12.944493,77.541539],[12.944554,77.541528],[12.944865,77.54146],[12.944922,77.541451],[12.945056,77.541465],[12.945479,77.541515],[12.94582,77.541577],[12.945907,77.541605],[12.946334,77.541746],[12.946828,77.54188],[12.947111,77.541954],[12.947396,77.542047]]},{"t":"r","c":[[12.932122,77.541411],[12.932065,77.541384],[12.931668,77.541194],[12.931317,77.541027],[12.931197,77.540953],[12.930717,77.54074],[12.930368,77.540586]]},{"t":"r","c":[[12.940304,77.545301],[12.940311,77.545227],[12.940362,77.544219],[12.940445,77.542695]]},{"t":"r","c":[[12.940375,77.539053],[12.940188,77.540418],[12.940137,77.5415],[12.940123,77.541858],[12.940075,77.542663],[12.940034,77.544209],[12.93997,77.545254],[12.939971,77.545322]]},{"t":"t","c":[[12.933529,77.539356],[12.933619,77.539186],[12.933759,77.538948]]},{"t":"t","c":[[12.933805,77.539265],[12.933721,77.539402]]},{"t":"r","c":[[12.932346,77.541711],[12.931858,77.543341]]},{"t":"r","c":[[12.93845,77.539263],[12.938428,77.539608],[12.938409,77.540103],[12.938405,77.540137],[12.938362,77.54061],[12.938226,77.541032],[12.938204,77.541145],[12.938192,77.541162],[12.938089,77.541271]]},{"t":"r","c":[[12.938089,77.541271],[12.938059,77.540967],[12.938043,77.540714],[12.938086,77.540238],[12.938112,77.539987],[12.938134,77.539751]]},{"t":"r","c":[[12.939713,77.541474],[12.940137,77.5415],[12.940521,77.541518]]},{"t":"r","c":[[12.94069,77.538875],[12.940707,77.539047],[12.940705,77.539175],[12.940696,77.539273],[12.940684,77.539369],[12.940587,77.539991],[12.940569,77.54012],[12.940521,77.541518]]},{"t":"r","c":[[12.937941,77.541246],[12.938089,77.541271],[12.938881,77.541366],[12.9393,77.541388]]},{"t":"r","c":[[12.933995,77.539982],[12.934017,77.54013],[12.933973,77.540714],[12.933904,77.541101]]},{"t":"r","c":[[12.933904,77.541101],[12.933748,77.541628],[12.933611,77.542036],[12.933591,77.5421]]},{"t":"r","c":[[12.941021,77.54133],[12.941894,77.541285],[12.942239,77.541261],[12.942662,77.541248],[12.943055,77.54122],[12.943429,77.541219],[12.943675,77.54119],[12.943794,77.541175]]},{"t":"t","c":[[12.929759,77.538942],[12.929837,77.539056],[12.930025,77.539311],[12.930058,77.539357],[12.930161,77.539514],[12.930196,77.539554],[12.930241,77.539603],[12.930464,77.539641],[12.930951,77.539699],[12.931316,77.539679],[12.931753,77.539666],[12.932107,77.539667],[12.932429,77.53964],[12.932763,77.539628],[12.933029,77.539572],[12.933354,77.539554],[12.933426,77.539577]]},{"t":"m","c":[[12.938516,77.531879],[12.93869,77.531955],[12.939163,77.532181],[12.939291,77.532246],[12.939469,77.532347],[12.939587,77.532406],[12.940259,77.53282],[12.940576,77.533029]]},{"t":"m","c":[[12.938516,77.531879],[12.938836,77.53185],[12.939023,77.531838],[12.939163,77.531804],[12.939297,77.531761],[12.939517,77.531668],[12.939868,77.531435],[12.940078,77.531326],[12.94023,77.531274],[12.940348,77.531253],[12.940435,77.531238],[12.940493,77.531237],[12.940555,77.53125],[12.940643,77.531291],[12.94091,77.531478],[12.94102,77.53152],[12.94113,77.531534],[12.941245,77.531538],[12.941394,77.531509]]},{"t":"m","c":[[12.940576,77.533029],[12.940685,77.533033],[12.940776,77.533032],[12.940859,77.533016],[12.940926,77.532996],[12.940999,77.532957],[12.94107,77.532899],[12.941121,77.532822],[12.941158,77.532734],[12.941179,77.53263],[12.941178,77.532532],[12.941145,77.532423],[12.941087,77.532329],[12.941006,77.532266],[12.940939,77.532228],[12.940864,77.5322],[12.940776,77.532187],[12.940692,77.532192],[12.94061,77.532205],[12.940534,77.53222],[12.940457,77.532233]]},{"t":"m","c":[[12.939589,77.532769],[12.939436,77.532933],[12.939374,77.533069],[12.939347,77.533224],[12.939371,77.533383],[12.939418,77.533488],[12.939492,77.533558],[12.939557,77.533605],[12.939654,77.533639],[12.93976,77.533655],[12.939862,77.533644],[12.939984,77.533608],[12.940103,77.533514],[12.940152,77.533432],[12.94018,77.533361],[12.940194,77.533295],[12.940196,77.533204],[12.940176,77.533067],[12.940132,77.532951],[12.940106,77.53288]]},{"t":"t","c":[[12.933777,77.532754],[12.933666,77.532723],[12.9336,77.532687],[12.933499,77.532608],[12.93345,77.532538],[12.933425,77.532485],[12.933413,77.532437],[12.933409,77.532396],[12.933409,77.532362],[12.933421,77.532291],[12.933432,77.532248],[12.933467,77.532158],[12.933572,77.531804],[12.933598,77.531699],[12.933613,77.5316]]},{"t":"t","c":[[12.93827,77.533336],[12.938027,77.533083],[12.937836,77.532896],[12.93779,77.532853],[12.937608,77.532737],[12.937546,77.532702],[12.937386,77.532639],[12.937279,77.532595],[12.937169,77.532558],[12.937058,77.532529],[12.936944,77.532507],[12.93683,77.532494],[12.936715,77.532489],[12.936597,77.532492],[12.936479,77.532503],[12.936363,77.532523],[12.936248,77.532551],[12.935821,77.532742],[12.935767,77.532765],[12.93553,77.532812],[12.935189,77.5328],[12.934953,77.532791],[12.934043,77.532788],[12.933906,77.53277]]},{"t":"m","c":[[12.941801,77.53437],[12.941626,77.53433],[12.941444,77.534211],[12.941024,77.533967],[12.940933,77.533927],[12.940848,77.533917],[12.939921,77.533881],[12.939274,77.533889],[12.939167,77.533871],[12.939073,77.533817],[12.938772,77.53354],[12.938692,77.533474],[12.938605,77.533446]]},{"t":"r","c":[[12.93796,77.53937],[12.937947,77.539223],[12.937955,77.538951],[12.937959,77.538893],[12.937974,77.538806],[12.938004,77.538772],[12.938034,77.538738],[12.939256,77.538518],[12.93993,77.538419],[12.940481,77.538362],[12.940608,77.538351],[12.940721,77.53834],[12.940838,77.538328],[12.940999,77.538312],[12.941071,77.538304],[12.94112,77.538298],[12.941211,77.538287],[12.94133,77.538272],[12.941452,77.538255],[12.941603,77.538237],[12.9419,77.53823],[12.943519,77.53819],[12.944351,77.538101],[12.944717,77.538087],[12.944943,77.538079],[12.94517,77.538071],[12.945403,77.538071]]},{"t":"t","c":[[12.933608,77.531193],[12.933593,77.531018],[12.93359,77.530943],[12.933588,77.530841],[12.933597,77.530737],[12.93363,77.530509],[12.933728,77.530076],[12.933753,77.529933],[12.933772,77.529815],[12.933783,77.529676],[12.933809,77.529262],[12.933826,77.529008],[12.933847,77.52857],[12.933851,77.528436],[12.933844,77.528394],[12.933829,77.528357],[12.933806,77.52833],[12.933774,77.52832],[12.933305,77.528325],[12.932785,77.528329],[12.932539,77.528336],[12.932075,77.528348],[12.931794,77.528355],[12.931756,77.528356],[12.931539,77.528362],[12.931479,77.528365],[12.931457,77.528366],[12.931344,77.528371],[12.929592,77.528496],[12.929532,77.528023],[12.929484,77.527711]]},{"t":"t","c":[[12.933613,77.5316],[12.933608,77.531193]]},{"t":"m","c":[[12.938784,77.533007],[12.938731,77.532839],[12.938641,77.532607],[12.938617,77.532523]]},{"t":"m","c":[[12.934715,77.531397],[12.934525,77.531407],[12.933976,77.531437]]},{"t":"m","c":[[12.940106,77.53288],[12.939812,77.532688],[12.939036,77.532273],[12.938809,77.532175],[12.938455,77.532015],[12.938085,77.531886],[12.937999,77.531862],[12.937817,77.531811],[12.936968,77.531769],[12.936612,77.53173]]},{"t":"r","c":[[12.941657,77.542212],[12.94098,77.542193],[12.940871,77.542189]]},{"t":"r","c":[[12.939253,77.542199],[12.937911,77.542085]]},{"t":"r","c":[[12.940521,77.541518],[12.940903,77.541508],[12.941012,77.541498]]},{"t":"r","c":[[12.941044,77.540018],[12.942309,77.539984]]},{"t":"r","c":[[12.9417,77.542617],[12.941311,77.542637],[12.940946,77.542636],[12.940847,77.542636]]},{"t":"r","c":[[12.938808,77.543024],[12.938728,77.544229]]},{"t":"r","c":[[12.942302,77.540347],[12.941906,77.54036],[12.941518,77.540375],[12.941074,77.540382],[12.940954,77.540385]]},{"t":"r","c":[[12.938081,77.54295],[12.937824,77.54398],[12.937732,77.544381],[12.93764,77.544799],[12.937628,77.544843]]},{"t":"r","c":[[12.939043,77.544755],[12.939139,77.544298],[12.9392,77.543064]]},{"t":"r","c":[[12.939522,77.54521],[12.939557,77.544525],[12.939629,77.543109],[12.939656,77.542627],[12.939695,77.541824],[12.939713,77.541474],[12.939841,77.538961]]},{"t":"r","c":[[12.939629,77.543109],[12.9392,77.543064],[12.938808,77.543024],[12.938081,77.54295],[12.937624,77.54289]]},{"t":"r","c":[[12.939235,77.534975],[12.93964,77.534921]]},{"t":"r","c":[[12.938687,77.534232],[12.938215,77.533704]]},{"t":"r","c":[[12.939129,77.534392],[12.939578,77.534337]]},{"t":"r","c":[[12.937947,77.539223],[12.93686,77.539133]]},{"t":"r","c":[[12.93897,77.540133],[12.938405,77.540137]]},{"t":"r","c":[[12.939368,77.535656],[12.939714,77.535612]]},{"t":"r","c":[[12.936643,77.538881],[12.936369,77.539718]]},{"t":"r","c":[[12.939327,77.535383],[12.939686,77.535352]]},{"t":"r","c":[[12.936158,77.53964],[12.936372,77.538811]]},{"t":"r","c":[[12.939607,77.534616],[12.939146,77.534688]]},{"t":"r","c":[[12.938027,77.533083],[12.938391,77.532674]]},{"t":"r","c":[[12.937955,77.538951],[12.936961,77.538915]]},{"t":"r","c":[[12.937901,77.539462],[12.937361,77.539386],[12.936779,77.53933]]},{"t":"r","c":[[12.93787,77.539683],[12.937188,77.539609],[12.936701,77.539551]]},{"t":"r","c":[[12.939901,77.53766],[12.939917,77.538178],[12.939918,77.538214],[12.93993,77.538419]]},{"t":"r","c":[[12.938017,77.53726],[12.937941,77.536859],[12.937867,77.536549],[12.93778,77.535798],[12.937694,77.535135],[12.937592,77.534343],[12.937587,77.534307],[12.937562,77.534058]]},{"t":"r","c":[[12.937836,77.532896],[12.938033,77.532699],[12.938182,77.532542],[12.938208,77.532505],[12.938212,77.532468],[12.938205,77.532433],[12.938192,77.532378],[12.938195,77.532331],[12.938226,77.532243],[12.938278,77.532094]]},{"t":"r","c":[[12.936316,77.538592],[12.936295,77.538792],[12.936372,77.538811],[12.936643,77.538881],[12.936924,77.538954]]},{"t":"r","c":[[12.938004,77.538772],[12.93793,77.538729],[12.937039,77.538719],[12.936347,77.53854],[12.936316,77.538592]]},{"t":"r","c":[[12.936621,77.539776],[12.936701,77.539551],[12.936779,77.53933],[12.93686,77.539133],[12.936924,77.538954],[12.936961,77.538915],[12.937039,77.538719]]},{"t":"r","c":[[12.939556,77.537543],[12.939624,77.537634],[12.939664,77.538228],[12.939918,77.538214]]},{"t":"r","c":[[12.93876,77.528432],[12.939089,77.528491],[12.93944,77.528561],[12.939586,77.528589],[12.93962,77.528593],[12.939891,77.528665],[12.940941,77.528982],[12.94128,77.529095],[12.941797,77.529196],[12.942442,77.529344],[12.942696,77.529392],[12.942792,77.52942],[12.942845,77.52947],[12.943032,77.529612],[12.943161,77.529639],[12.943284,77.52953]]},{"t":"r","c":[[12.937596,77.528502],[12.93749,77.528845],[12.937436,77.529021],[12.937282,77.529781]]},{"t":"r","c":[[12.939121,77.530669],[12.938268,77.530811]]},{"t":"r","c":[[12.93757,77.529842],[12.937714,77.529102],[12.937867,77.528597]]},{"t":"r","c":[[12.937983,77.529558],[12.937919,77.529915]]},{"t":"r","c":[[12.939189,77.531071],[12.938314,77.531195]]},{"t":"r","c":[[12.939628,77.530161],[12.939049,77.530247],[12.938187,77.530391]]},{"t":"r","c":[[12.938122,77.528679],[12.938017,77.529107],[12.938048,77.529179]]},{"t":"r","c":[[12.93962,77.528593],[12.939562,77.528943],[12.939583,77.529293],[12.939617,77.529705]]},{"t":"r","c":[[12.939891,77.528665],[12.940092,77.528154],[12.940798,77.528389],[12.941085,77.528499],[12.940941,77.528982]]},{"t":"r","c":[[12.939792,77.530145],[12.939891,77.529392],[12.940143,77.529436],[12.940427,77.529485],[12.940719,77.529536],[12.940637,77.530236],[12.940329,77.530213],[12.940047,77.530192],[12.939628,77.530161]]},{"t":"r","c":[[12.938104,77.529953],[12.937919,77.529915],[12.937708,77.529871],[12.937621,77.529853],[12.93757,77.529842],[12.937369,77.529799],[12.937282,77.529781],[12.936988,77.529719],[12.936675,77.529653],[12.936317,77.529577],[12.936276,77.529541],[12.936309,77.528866]]},{"t":"r","c":[[12.939562,77.528943],[12.938841,77.529077],[12.938048,77.529179],[12.937983,77.529558],[12.938905,77.529435],[12.939583,77.529293]]},{"t":"r","c":[[12.939263,77.53151],[12.938403,77.53152],[12.938348,77.531406],[12.938314,77.531195],[12.938268,77.530811],[12.938187,77.530391],[12.938104,77.529953],[12.938973,77.529841],[12.939617,77.529705]]},{"t":"t","c":[[12.934114,77.538177],[12.934435,77.537657],[12.934603,77.537369],[12.934739,77.53714],[12.93491,77.536857],[12.935071,77.536601],[12.935087,77.536569],[12.935146,77.536458],[12.935201,77.536355],[12.93577,77.535253],[12.935942,77.534918],[12.936048,77.534724],[12.936093,77.534647],[12.93617,77.534514],[12.936385,77.53426],[12.936626,77.534052],[12.937114,77.533892],[12.93747,77.533778],[12.937552,77.533751],[12.937647,77.533707],[12.937889,77.533561],[12.937961,77.533521],[12.93827,77.533336]]},{"t":"r","c":[[12.929382,77.539794],[12.929945,77.539969],[12.930553,77.540246],[12.930843,77.540367],[12.931704,77.540719],[12.93195,77.540825],[12.932349,77.540985],[12.932406,77.541009]]},{"t":"r","c":[[12.936616,77.544049],[12.936191,77.54393]]},{"t":"t","c":[[12.944033,77.530145],[12.943936,77.530199],[12.943228,77.530623],[12.942939,77.530788],[12.942427,77.531054],[12.94224,77.531158],[12.941928,77.531329],[12.941588,77.531517],[12.94114,77.531792],[12.940965,77.531905]]},{"t":"m","c":[[12.939937,77.532384],[12.939983,77.532309],[12.940012,77.532263],[12.940045,77.532196],[12.940078,77.532099],[12.940088,77.532021],[12.940075,77.531956],[12.940053,77.531883],[12.940025,77.531832],[12.939989,77.531794],[12.939928,77.531744],[12.939855,77.531716],[12.939772,77.531702],[12.939714,77.531704],[12.939636,77.531722],[12.939577,77.531747],[12.939559,77.531762],[12.939526,77.53179],[12.939474,77.531855],[12.939442,77.531922],[12.939428,77.531999],[12.939431,77.532084],[12.939454,77.53216],[12.939483,77.53224],[12.93954,77.532337],[12.939587,77.532406]]},{"t":"t","c":[[12.939405,77.532713],[12.939875,77.532422]]},{"t":"r","c":[[12.938418,77.531726],[12.938403,77.53152]]},{"t":"r","c":[[12.938278,77.532094],[12.938418,77.531726]]},{"t":"r","c":[[12.935071,77.536601],[12.935064,77.536601],[12.935028,77.536598],[12.934564,77.536558],[12.934202,77.536523]]},{"t":"r","c":[[12.932757,77.536564],[12.93244,77.536475]]},{"t":"r","c":[[12.932757,77.536564],[12.932681,77.536846],[12.932626,77.537045],[12.932901,77.537124]]},{"t":"r","c":[[12.932681,77.536846],[12.932399,77.53677],[12.93208,77.537798]]},{"t":"r","c":[[12.932901,77.537124],[12.933004,77.536768],[12.93305,77.53659],[12.933118,77.536349]]},{"t":"r","c":[[12.932757,77.536564],[12.93283,77.536284],[12.932911,77.535981],[12.933193,77.536062],[12.933118,77.536349]]},{"t":"r","c":[[12.934127,77.537074],[12.933761,77.537031],[12.93369,77.537029],[12.933634,77.537021],[12.933343,77.536984],[12.933218,77.536972]]},{"t":"r","c":[[12.932764,77.538111],[12.932901,77.537124]]},{"t":"r","c":[[12.932764,77.538111],[12.933009,77.538117],[12.933084,77.538119],[12.93329,77.538124],[12.933358,77.538125],[12.933457,77.538128],[12.934017,77.538142],[12.934114,77.538177]]},{"t":"r","c":[[12.934228,77.53625],[12.934257,77.535976]]},{"t":"r","c":[[12.934095,77.537334],[12.933702,77.537306]]},{"t":"r","c":[[12.933702,77.537306],[12.933706,77.537594]]},{"t":"r","c":[[12.933236,77.53719],[12.933375,77.537489]]},{"t":"r","c":[[12.933236,77.53719],[12.933121,77.537167],[12.932901,77.537124]]},{"t":"r","c":[[12.933121,77.537167],[12.933084,77.538119]]},{"t":"r","c":[[12.934228,77.53625],[12.93391,77.536213],[12.93364,77.536183],[12.933193,77.536062]]},{"t":"r","c":[[12.93283,77.536284],[12.932151,77.536097]]},{"t":"r","c":[[12.932911,77.535981],[12.932096,77.535749]]},{"t":"r","c":[[12.932911,77.535981],[12.932974,77.535692],[12.932912,77.535674],[12.932234,77.535478]]},{"t":"r","c":[[12.932764,77.538111],[12.932699,77.538103],[12.932464,77.538076],[12.932385,77.538067],[12.932116,77.538037],[12.932127,77.53785],[12.932132,77.537824],[12.93208,77.537798],[12.932018,77.537797],[12.931997,77.537815],[12.931976,77.537938],[12.931957,77.538014],[12.931837,77.537992],[12.931853,77.537771],[12.931876,77.537686],[12.931957,77.53743],[12.931976,77.537375],[12.932074,77.537097],[12.932262,77.536655],[12.932324,77.536524]]},{"t":"r","c":[[12.932074,77.537097],[12.931805,77.537035],[12.931496,77.536964],[12.931371,77.536935],[12.931067,77.536865],[12.930677,77.536783]]},{"t":"r","c":[[12.929521,77.535173],[12.929986,77.535202],[12.930266,77.53522],[12.931263,77.535283],[12.931977,77.535351]]},{"t":"r","c":[[12.935477,77.543643],[12.935832,77.543788],[12.936191,77.54393]]},{"t":"r","c":[[12.936927,77.543056],[12.936518,77.54294],[12.936165,77.542839],[12.936282,77.542408]]},{"t":"r","c":[[12.936165,77.542839],[12.935974,77.5434],[12.935832,77.543788]]},{"t":"r","c":[[12.937127,77.542419],[12.937583,77.542447]]},{"t":"r","c":[[12.93448,77.53988],[12.935566,77.540169]]},{"t":"r","c":[[12.935566,77.540169],[12.936012,77.540288]]},{"t":"r","c":[[12.936012,77.540288],[12.936608,77.54051],[12.936799,77.540581],[12.937577,77.54087]]},{"t":"r","c":[[12.93448,77.53988],[12.934372,77.539867],[12.93428,77.539857],[12.934019,77.539934],[12.933995,77.539982]]},{"t":"r","c":[[12.93448,77.53988],[12.934546,77.539544],[12.934562,77.539458],[12.934592,77.539294],[12.934609,77.539197]]},{"t":"r","c":[[12.932102,77.541592],[12.932149,77.541633],[12.932346,77.541711],[12.932723,77.541836],[12.933149,77.54196],[12.933519,77.542077]]},{"t":"r","c":[[12.932723,77.541836],[12.933002,77.541316],[12.933423,77.540728],[12.933493,77.540651],[12.933765,77.540219],[12.93394,77.540023],[12.933995,77.539982]]},{"t":"m","c":[[12.93651,77.531714],[12.935687,77.531577],[12.934715,77.531397]]},{"t":"m","c":[[12.936612,77.53173],[12.936552,77.531721],[12.93651,77.531714]]},{"t":"m","c":[[12.936595,77.531184],[12.936639,77.531188],[12.936697,77.5312]]},{"t":"m","c":[[12.934724,77.531283],[12.93636,77.531161],[12.936595,77.531184]]},{"t":"r","c":[[12.937436,77.529021],[12.937714,77.529102],[12.938048,77.529179]]},{"t":"r","c":[[12.937049,77.52831],[12.936945,77.528614]]},{"t":"r","c":[[12.935852,77.526508],[12.935835,77.526546],[12.935772,77.526784],[12.93571,77.527057],[12.935639,77.527369],[12.935583,77.527615],[12.935542,77.527804],[12.935381,77.528828]]},{"t":"r","c":[[12.939089,77.528491],[12.93917,77.527957]]},{"t":"r","c":[[12.93944,77.528561],[12.939483,77.528026]]},{"t":"r","c":[[12.939664,77.538228],[12.939441,77.538261],[12.939206,77.538296]]},{"t":"r","c":[[12.940159,77.533969],[12.940212,77.534633],[12.940246,77.534976],[12.940295,77.535462],[12.940362,77.535959]]},{"t":"r","c":[[12.937624,77.54289],[12.937539,77.543351],[12.937262,77.544658]]},{"t":"t","c":[[12.940636,77.54719],[12.940606,77.547066],[12.940625,77.546942],[12.940643,77.546624],[12.940686,77.54574],[12.940712,77.54534],[12.940712,77.545276],[12.940714,77.545201],[12.940724,77.54492],[12.940756,77.544212],[12.940843,77.542729],[12.940847,77.542636],[12.940871,77.542189],[12.940894,77.541744],[12.940903,77.541508],[12.940954,77.540385],[12.94096,77.54027],[12.941029,77.540187]]},{"t":"r","c":[[12.933748,77.541628],[12.934248,77.541808]]},{"t":"r","c":[[12.938275,77.53421],[12.937946,77.533874]]},{"t":"r","c":[[12.938753,77.534232],[12.938761,77.534615],[12.938893,77.535663],[12.938968,77.536271],[12.939021,77.53674]]},{"t":"r","c":[[12.940437,77.53777],[12.940445,77.538002],[12.940454,77.538045],[12.940476,77.538173],[12.940481,77.538362]]},{"t":"r","c":[[12.939917,77.538178],[12.940074,77.538173],[12.940192,77.538171],[12.940476,77.538173]]},{"t":"r","c":[[12.940659,77.537288],[12.940633,77.537467],[12.940584,77.537663],[12.940575,77.537757],[12.940575,77.537771],[12.940598,77.537917],[12.9406,77.538021],[12.940599,77.538145],[12.940608,77.538351]]},{"t":"r","c":[[12.940074,77.538173],[12.940045,77.537832],[12.940172,77.537813],[12.940437,77.53777],[12.940575,77.537757]]},{"t":"r","c":[[12.940876,77.537245],[12.940879,77.537363],[12.940881,77.537486],[12.940902,77.537597],[12.940916,77.53768],[12.94092,77.537707],[12.940934,77.537813],[12.940942,77.537987],[12.940973,77.538162],[12.940999,77.538312]]},{"t":"r","c":[[12.935697,77.539915],[12.936526,77.540224],[12.936882,77.540357],[12.937638,77.540638]]},{"t":"t","c":[[12.93796,77.53937],[12.937901,77.539462],[12.93788,77.539616],[12.937877,77.539636],[12.93787,77.539683],[12.937851,77.539822],[12.937787,77.539914],[12.937537,77.539906],[12.937508,77.539901],[12.937232,77.539855],[12.936621,77.539776],[12.936554,77.539763],[12.936369,77.539718],[12.936158,77.53964],[12.935733,77.539489],[12.934694,77.539217],[12.934609,77.539197],[12.934138,77.539087],[12.934124,77.539084],[12.933937,77.539038]]},{"t":"t","c":[[12.943539,77.538685],[12.943189,77.538716],[12.94111,77.538828],[12.941078,77.538828],[12.94069,77.538875],[12.9404,77.538898],[12.939841,77.538961],[12.939435,77.539023],[12.939253,77.539056],[12.939041,77.539112],[12.938822,77.539165],[12.93845,77.539263],[12.93796,77.53937]]},{"t":"r","c":[[12.935201,77.536355],[12.935279,77.536394],[12.935365,77.536434],[12.935458,77.536469]]},{"t":"t","c":[[12.935225,77.536498],[12.935146,77.536458]]},{"t":"t","c":[[12.933759,77.538948],[12.934121,77.538347],[12.934197,77.53823]]},{"t":"t","c":[[12.933413,77.532437],[12.933383,77.532424],[12.933364,77.532416],[12.933124,77.532301],[12.933084,77.532288],[12.933039,77.532274],[12.932979,77.532264],[12.932919,77.532258],[12.932854,77.532255],[12.932804,77.532258],[12.932673,77.532275],[12.932401,77.532346],[12.932233,77.53241],[12.932175,77.532436],[12.932098,77.532494],[12.932079,77.532513],[12.932054,77.532566],[12.932037,77.532615],[12.932009,77.532843],[12.931996,77.532968],[12.931985,77.533045],[12.931969,77.533114],[12.931933,77.533181],[12.931886,77.533219],[12.931856,77.533239],[12.931718,77.53328],[12.931464,77.533374],[12.931385,77.533399],[12.931164,77.533482],[12.931093,77.533515],[12.930913,77.533605],[12.930856,77.533647],[12.930746,77.533732],[12.930601,77.533853],[12.930374,77.534051],[12.930219,77.534255],[12.93018,77.534297],[12.929946,77.534453],[12.929709,77.53461],[12.929552,77.534718]]},{"t":"t","c":[[12.933906,77.53277],[12.933777,77.532754]]},{"t":"m","c":[[12.933349,77.531484],[12.932346,77.531554],[12.931919,77.531592],[12.930625,77.531689],[12.928242,77.531866]]},{"t":"m","c":[[12.933976,77.531437],[12.933349,77.531484]]},{"t":"m","c":[[12.933958,77.531321],[12.934724,77.531283]]},{"t":"m","c":[[12.933341,77.531366],[12.933506,77.531354],[12.933754,77.531336],[12.933958,77.531321]]},{"t":"m","c":[[12.940576,77.533029],[12.940699,77.533133],[12.941301,77.533707],[12.941888,77.534279],[12.942186,77.534569],[12.943135,77.53563],[12.943817,77.536195],[12.944789,77.536878]]},{"t":"m","c":[[12.944759,77.537012],[12.944222,77.536629],[12.943725,77.536275],[12.943065,77.5357],[12.942512,77.535105],[12.94239,77.534973],[12.942141,77.534706],[12.941989,77.534566],[12.941801,77.53437],[12.941247,77.533794],[12.940551,77.533211],[12.940106,77.53288]]},{"t":"t","c":[[12.934718,77.537573],[12.934665,77.53766]]},{"t":"t","c":[[12.933133,77.54015],[12.933023,77.540308]]},{"t":"t","c":[[12.934164,77.538509],[12.933797,77.539131],[12.93364,77.539406]]},{"t":"t","c":[[12.93708,77.534221],[12.936718,77.534346],[12.93659,77.534453],[12.936381,77.534691],[12.9362,77.535011],[12.936069,77.535275],[12.935932,77.535531],[12.935815,77.53575],[12.935693,77.535992],[12.935656,77.536065],[12.935458,77.536469],[12.935205,77.536941],[12.935039,77.537233],[12.934894,77.537477]]},{"t":"t","c":[[12.938215,77.533704],[12.938063,77.5338],[12.937946,77.533874],[12.937791,77.533966],[12.937562,77.534058],[12.93708,77.534221]]},{"t":"t","c":[[12.939454,77.532849],[12.939306,77.533002],[12.938605,77.533446],[12.938495,77.533519],[12.938215,77.533704]]},{"t":"r","c":[[12.937819,77.534308],[12.937829,77.534342],[12.93788,77.534949],[12.937998,77.535762],[12.938095,77.536532]]},{"t":"t","c":[[12.933937,77.539038],[12.933902,77.539101],[12.933887,77.539127],[12.933809,77.539257],[12.933805,77.539265]]},{"t":"t","c":[[12.933937,77.539038],[12.933815,77.538995]]},{"t":"t","c":[[12.934114,77.538177],[12.934197,77.53823]]},{"t":"t","c":[[12.931405,77.542371],[12.931995,77.541582],[12.932122,77.541411],[12.932406,77.541009],[12.93287,77.540381],[12.933356,77.539693],[12.933426,77.539577],[12.933529,77.539356]]},{"t":"t","c":[[12.933023,77.540308],[12.932224,77.541424],[12.932102,77.541592],[12.932061,77.541642],[12.931451,77.542453]]},{"t":"r","c":[[12.936292,77.536042],[12.936279,77.53565]]},{"t":"r","c":[[12.935693,77.535992],[12.935779,77.536029],[12.935842,77.536056],[12.935875,77.536112],[12.935886,77.536121],[12.935901,77.536126],[12.935915,77.536126],[12.936292,77.536042],[12.936552,77.535984],[12.936568,77.535979],[12.93658,77.535969],[12.936585,77.535958],[12.936587,77.53594],[12.936574,77.535655],[12.936569,77.535631],[12.936558,77.535619],[12.936543,77.53561],[12.936514,77.535609],[12.936279,77.53565],[12.936026,77.53572],[12.93601,77.535728],[12.935996,77.535741],[12.935842,77.536056]]},{"t":"r","c":[[12.931847,77.540502],[12.931704,77.540719]]},{"t":"r","c":[[12.932429,77.53964],[12.931847,77.540502],[12.931623,77.540394],[12.931348,77.540265],[12.931044,77.54011],[12.931316,77.539679]]},{"t":"r","c":[[12.932763,77.539628],[12.93195,77.540825]]},{"t":"r","c":[[12.93624,77.536479],[12.936437,77.537401]]},{"t":"r","c":[[12.936067,77.53747],[12.935077,77.537649],[12.935021,77.537656],[12.934942,77.537656],[12.9348,77.537631]]},{"t":"r","c":[[12.937378,77.537166],[12.937467,77.537416]]},{"t":"r","c":[[12.937038,77.536487],[12.937111,77.537101]]},{"t":"r","c":[[12.931466,77.52865],[12.931482,77.528949],[12.931711,77.528979],[12.931729,77.528991],[12.931709,77.529148],[12.93136,77.529085],[12.931336,77.529068],[12.931328,77.529034],[12.931331,77.528932],[12.931343,77.528655]]},{"t":"r","c":[[12.932186,77.543465],[12.931991,77.543992]]},{"t":"r","c":[[12.938776,77.528747],[12.939586,77.528589]]},{"t":"r","c":[[12.936752,77.528539],[12.936945,77.528614],[12.93749,77.528845]]},{"t":"r","c":[[12.931982,77.529145],[12.933021,77.529209],[12.933809,77.529262]]},{"t":"r","c":[[12.932673,77.532275],[12.932656,77.531901],[12.932636,77.531888],[12.932605,77.531886],[12.932342,77.531928],[12.932101,77.531947],[12.931964,77.531919],[12.931641,77.531964],[12.931565,77.531982],[12.93147,77.532014],[12.931314,77.532077],[12.93124,77.532111],[12.931148,77.532135],[12.930971,77.53217],[12.930729,77.532218],[12.930448,77.532279],[12.9302,77.532323],[12.92998,77.532362],[12.929923,77.5324],[12.929918,77.53246],[12.930047,77.533166],[12.930219,77.534255]]},{"t":"r","c":[[12.931718,77.53328],[12.931664,77.532638],[12.931565,77.531982]]},{"t":"r","c":[[12.932098,77.532494],[12.932067,77.532466],[12.931964,77.531919]]},{"t":"r","c":[[12.932476,77.53319],[12.932446,77.532926],[12.932429,77.53265],[12.932401,77.532346]]},{"t":"r","c":[[12.930265,77.53434],[12.930551,77.534266],[12.930635,77.534247],[12.93072,77.534239],[12.930927,77.534233],[12.931216,77.53422],[12.931313,77.534216],[12.931496,77.534212],[12.931689,77.534207]]},{"t":"r","c":[[12.931989,77.533822],[12.932011,77.534155]]},{"t":"r","c":[[12.932562,77.533457],[12.931941,77.533521]]},{"t":"r","c":[[12.931989,77.533822],[12.932727,77.533735],[12.932775,77.533719],[12.932792,77.533675],[12.932782,77.533429],[12.932772,77.533162]]},{"t":"r","c":[[12.933338,77.533721],[12.933326,77.533207],[12.933287,77.533176],[12.93325,77.53317],[12.933049,77.533157],[12.932772,77.533162],[12.932476,77.53319],[12.93195,77.533242],[12.931886,77.533219]]},{"t":"r","c":[[12.932446,77.532926],[12.933364,77.53281]]},{"t":"r","c":[[12.932429,77.53265],[12.933152,77.532605]]},{"t":"r","c":[[12.933049,77.533157],[12.933071,77.533882]]},{"t":"r","c":[[12.933642,77.533581],[12.933645,77.533239],[12.933656,77.532811],[12.933666,77.532723]]},{"t":"r","c":[[12.931923,77.535607],[12.932027,77.535702]]},{"t":"r","c":[[12.932027,77.535702],[12.932096,77.535749]]},{"t":"r","c":[[12.935189,77.5328],[12.935137,77.53364],[12.935139,77.533646],[12.935142,77.533652],[12.935149,77.533656],[12.935439,77.533682],[12.93553,77.532812]]},{"t":"r","c":[[12.940403,77.538641],[12.941077,77.538563]]},{"t":"r","c":[[12.939411,77.537732],[12.939441,77.538261]]},{"t":"r","c":[[12.937451,77.535841],[12.937341,77.535855],[12.937322,77.535871],[12.937316,77.535907],[12.937367,77.536216]]},{"t":"r","c":[[12.937009,77.536105],[12.937038,77.536487]]},{"t":"r","c":[[12.936625,77.536212],[12.936647,77.536476]]},{"t":"r","c":[[12.936429,77.536477],[12.936622,77.537137]]},{"t":"r","c":[[12.935946,77.536481],[12.936144,77.537362],[12.936134,77.537416],[12.936067,77.53747]]},{"t":"r","c":[[12.934605,77.537912],[12.934778,77.537937],[12.934853,77.537948],[12.934949,77.537946],[12.935446,77.537917],[12.935818,77.537895]]},{"t":"r","c":[[12.935039,77.537233],[12.935221,77.537268],[12.935538,77.537251]]},{"t":"r","c":[[12.935205,77.536941],[12.935249,77.536967],[12.935915,77.536893]]},{"t":"r","c":[[12.934906,77.536005],[12.934257,77.535976]]},{"t":"r","c":[[12.93339,77.53671],[12.93367,77.536735],[12.934167,77.536789]]},{"t":"r","c":[[12.934603,77.537369],[12.934095,77.537334]]},{"t":"r","c":[[12.931941,77.536365],[12.931805,77.537035]]},{"t":"r","c":[[12.932136,77.536422],[12.931941,77.536365],[12.931674,77.536306],[12.931644,77.53631],[12.931623,77.536334],[12.931496,77.536964]]},{"t":"r","c":[[12.931224,77.535953],[12.93145,77.535958],[12.931845,77.535966]]},{"t":"r","c":[[12.931658,77.537295],[12.931499,77.537261],[12.931328,77.537213],[12.930645,77.537116],[12.930419,77.537099]]},{"t":"r","c":[[12.932364,77.538893],[12.932658,77.538899],[12.932971,77.538914],[12.933307,77.538928]]},{"t":"r","c":[[12.932408,77.539054],[12.933232,77.539073],[12.933255,77.539068],[12.933284,77.539041],[12.933307,77.538928],[12.933307,77.538776],[12.933291,77.538746],[12.933257,77.538737],[12.932969,77.538733],[12.932673,77.538725],[12.932412,77.538716]]},{"t":"r","c":[[12.932239,77.538908],[12.931094,77.538752]]},{"t":"r","c":[[12.932093,77.538662],[12.93113,77.538537]]},{"t":"r","c":[[12.931067,77.538953],[12.932225,77.539078]]},{"t":"r","c":[[12.930987,77.539396],[12.931654,77.539423],[12.932897,77.539366],[12.93303,77.539355],[12.933291,77.539333]]},{"t":"r","c":[[12.931654,77.539423],[12.931652,77.539244],[12.931663,77.539227],[12.931701,77.539223],[12.932201,77.53923]]},{"t":"r","c":[[12.931167,77.538332],[12.93143,77.538318],[12.931644,77.538365]]},{"t":"r","c":[[12.931753,77.539666],[12.931348,77.540265]]},{"t":"r","c":[[12.931623,77.540394],[12.932107,77.539667]]},{"t":"r","c":[[12.933253,77.543111],[12.933774,77.543276],[12.934138,77.543394]]},{"t":"r","c":[[12.936505,77.541131],[12.93743,77.541411]]},{"t":"r","c":[[12.936955,77.540104],[12.937386,77.540254],[12.937709,77.540366]]},{"t":"r","c":[[12.93726,77.543447],[12.937583,77.542447]]},{"t":"r","c":[[12.933548,77.542238],[12.933253,77.543111],[12.933064,77.543671],[12.932845,77.544318]]},{"t":"r","c":[[12.936979,77.543899],[12.936872,77.544511]]},{"t":"r","c":[[12.93628,77.544368],[12.936545,77.544244],[12.93659,77.544213],[12.936631,77.544167],[12.936651,77.544135],[12.936895,77.543734],[12.936925,77.543703],[12.936972,77.543683],[12.937047,77.543675],[12.937105,77.543659],[12.937185,77.543609],[12.937418,77.543468],[12.937446,77.543423],[12.93746,77.543373],[12.937589,77.542883],[12.937702,77.542458]]},{"t":"r","c":[[12.939829,77.529168],[12.939891,77.528665]]},{"t":"r","c":[[12.937578,77.536517],[12.937674,77.537365]]},{"t":"r","c":[[12.937378,77.537166],[12.937352,77.536933],[12.937352,77.536525]]},{"t":"r","c":[[12.932484,77.5377],[12.932563,77.537208],[12.932626,77.537045]]},{"t":"r","c":[[12.932324,77.536524],[12.932373,77.536482],[12.93244,77.536475]]},{"t":"r","c":[[12.932234,77.535478],[12.932133,77.535435]]},{"t":"r","c":[[12.932136,77.536422],[12.932208,77.536451]]},{"t":"r","c":[[12.931762,77.537261],[12.931805,77.537035]]},{"t":"r","c":[[12.932699,77.538103],[12.932661,77.538443],[12.932647,77.53861]]},{"t":"r","c":[[12.933009,77.538117],[12.932966,77.538505],[12.932959,77.538618]]},{"t":"r","c":[[12.933619,77.539186],[12.933451,77.539203]]},{"t":"r","c":[[12.939964,77.535003],[12.940246,77.534976]]},{"t":"r","c":[[12.935832,77.543788],[12.935745,77.544027]]},{"t":"r","c":[[12.931837,77.53421],[12.932011,77.534155]]},{"t":"r","c":[[12.931732,77.537346],[12.931762,77.537261]]},{"t":"r","c":[[12.932186,77.543465],[12.932633,77.543521]]},{"t":"r","c":[[12.932011,77.534155],[12.932549,77.534067]]},{"t":"r","c":[[12.931709,77.529148],[12.931677,77.529291],[12.931987,77.529369]]},{"t":"m","c":[[12.939559,77.531762],[12.939517,77.531668]]},{"t":"r","c":[[12.937839,77.528587],[12.937953,77.528282]]},{"t":"r","c":[[12.938122,77.528679],[12.938246,77.528348]]},{"t":"r","c":[[12.940427,77.529485],[12.940329,77.530213]]},{"t":"r","c":[[12.940143,77.529436],[12.940047,77.530192]]},{"t":"r","c":[[12.939662,77.535123],[12.939285,77.535198]]},{"t":"r","c":[[12.940246,77.534976],[12.940566,77.534949]]},{"t":"r","c":[[12.940575,77.537771],[12.940673,77.537764],[12.940721,77.53834]]},{"t":"r","c":[[12.940172,77.537813],[12.940192,77.538171]]},{"t":"r","c":[[12.938033,77.532699],[12.937816,77.532451]]},{"t":"r","c":[[12.938938,77.540582],[12.938706,77.540568],[12.938527,77.540622],[12.938362,77.54061]]},{"t":"r","c":[[12.939017,77.539565],[12.938428,77.539608]]},{"t":"r","c":[[12.94112,77.538298],[12.941065,77.537829],[12.941007,77.537532],[12.94097,77.537292]]},{"t":"r","c":[[12.940757,77.537213],[12.940786,77.537485],[12.940817,77.53772],[12.940819,77.53792],[12.940838,77.538328]]},{"t":"r","c":[[12.931644,77.53631],[12.931682,77.536121]]},{"t":"r","c":[[12.932775,77.533719],[12.932795,77.534002]]},{"t":"r","c":[[12.933364,77.53281],[12.933391,77.532864],[12.933409,77.533131]]},{"t":"r","c":[[12.933364,77.53281],[12.933333,77.532546],[12.93334,77.532518],[12.933355,77.53248],[12.933383,77.532424]]},{"t":"r","c":[[12.932401,77.532346],[12.932342,77.531928]]},{"t":"r","c":[[12.933656,77.532811],[12.933902,77.532979],[12.933908,77.533529],[12.933642,77.533581]]},{"t":"r","c":[[12.934043,77.532788],[12.934108,77.533447]]},{"t":"r","c":[[12.93367,77.536735],[12.933634,77.537021]]},{"t":"r","c":[[12.934078,77.537627],[12.934435,77.537657]]},{"t":"r","c":[[12.933358,77.538125],[12.933375,77.537489]]},{"t":"r","c":[[12.933004,77.536768],[12.933132,77.536803]]},{"t":"r","c":[[12.933457,77.538128],[12.933421,77.538637]]},{"t":"r","c":[[12.93329,77.538124],[12.933272,77.538606]]},{"t":"r","c":[[12.932385,77.538067],[12.932337,77.538458]]},{"t":"r","c":[[12.931853,77.537771],[12.931734,77.53776]]},{"t":"r","c":[[12.931837,77.537992],[12.931777,77.537987]]},{"t":"r","c":[[12.935623,77.539942],[12.935566,77.540169]]},{"t":"r","c":[[12.937609,77.535819],[12.937625,77.536131],[12.937617,77.536223],[12.937646,77.536524]]},{"t":"r","c":[[12.937367,77.536216],[12.93742,77.536523]]},{"t":"r","c":[[12.937467,77.537416],[12.937047,77.53746]]},{"t":"r","c":[[12.93303,77.539355],[12.933029,77.539572]]},{"t":"r","c":[[12.931756,77.528356],[12.931779,77.528722]]},{"t":"r","c":[[12.936988,77.529719],[12.937031,77.529438],[12.937137,77.528968]]},{"t":"r","c":[[12.937708,77.529871],[12.937738,77.530212]]},{"t":"r","c":[[12.937369,77.529799],[12.937395,77.530346]]},{"t":"r","c":[[12.936367,77.528399],[12.936752,77.528539]]},{"t":"r","c":[[12.933409,77.533131],[12.933369,77.533178],[12.933326,77.533207]]},{"t":"r","c":[[12.936316,77.538592],[12.935577,77.538462],[12.935486,77.538423],[12.935458,77.538337],[12.935446,77.537917]]},{"t":"m","c":[[12.936968,77.531769],[12.93706,77.53127]]},{"t":"t","c":[[12.939454,77.532849],[12.93925,77.532926],[12.939076,77.533038]]},{"t":"t","c":[[12.939031,77.532967],[12.939208,77.532848],[12.939405,77.532713]]},{"t":"t","c":[[12.939076,77.533038],[12.938853,77.533185],[12.938766,77.53324],[12.938414,77.533468],[12.937866,77.533794],[12.937733,77.533875],[12.937615,77.533927],[12.937009,77.534124],[12.936713,77.534219],[12.936519,77.534391]]},{"t":"t","c":[[12.936458,77.534323],[12.936671,77.534137],[12.936983,77.534036],[12.937583,77.533841],[12.937692,77.533793],[12.937821,77.533714],[12.938367,77.533389],[12.938719,77.533162],[12.938807,77.533107],[12.939031,77.532967]]},{"t":"r","c":[[12.933307,77.538928],[12.933623,77.538921]]},{"t":"t","c":[[12.94111,77.538828],[12.941107,77.539042],[12.941097,77.539278],[12.941073,77.539666],[12.941044,77.540018],[12.941029,77.540187]]},{"t":"t","c":[[12.940998,77.541744],[12.94098,77.542193],[12.940946,77.542636],[12.940817,77.54487]]},{"t":"r","c":[[12.932071,77.534307],[12.932093,77.534338],[12.932406,77.534796]]},{"t":"r","c":[[12.932011,77.534155],[12.932042,77.534232],[12.932071,77.534307]]},{"t":"r","c":[[12.93788,77.539616],[12.938125,77.539688]]},{"t":"r","c":[[12.933064,77.543671],[12.932633,77.543521]]},{"t":"r","c":[[12.932585,77.528698],[12.931779,77.528722]]},{"t":"r","c":[[12.937819,77.534308],[12.937777,77.534297],[12.937587,77.534307]]},{"t":"r","c":[[12.938059,77.540967],[12.937973,77.540764],[12.93796,77.540738]]},{"t":"r","c":[[12.933291,77.539333],[12.93344,77.539333],[12.933475,77.539337],[12.933529,77.539356]]},{"t":"r","c":[[12.938093,77.537701],[12.938017,77.53726]]},{"t":"t","c":[[12.941029,77.540187],[12.941076,77.540267],[12.941074,77.540382],[12.941021,77.54133],[12.941012,77.541498],[12.940998,77.541744]]},{"t":"r","c":[[12.934017,77.538142],[12.934078,77.537627],[12.934095,77.537334]]},{"t":"r","c":[[12.934739,77.53714],[12.934127,77.537074]]},{"t":"r","c":[[12.934127,77.537074],[12.934167,77.536789]]},{"t":"r","c":[[12.934167,77.536789],[12.93491,77.536857]]},{"t":"r","c":[[12.934167,77.536789],[12.934202,77.536523]]},{"t":"r","c":[[12.934202,77.536523],[12.934093,77.536512],[12.933824,77.536483],[12.933445,77.53644],[12.933118,77.536349]]},{"t":"r","c":[[12.934202,77.536523],[12.934228,77.53625]]},{"t":"r","c":[[12.935028,77.536598],[12.935001,77.536317],[12.934228,77.53625]]},{"t":"r","c":[[12.933702,77.537306],[12.93369,77.537029]]},{"t":"r","c":[[12.934095,77.537334],[12.934127,77.537074]]},{"t":"r","c":[[12.933343,77.536984],[12.93339,77.53671]]},{"t":"r","c":[[12.931856,77.533239],[12.931941,77.533521]]},{"t":"r","c":[[12.931941,77.533521],[12.931669,77.533579]]},{"t":"r","c":[[12.931941,77.533521],[12.931989,77.533822]]},{"t":"r","c":[[12.931703,77.533858],[12.931989,77.533822]]},{"t":"r","c":[[12.933364,77.53281],[12.933482,77.532811],[12.933543,77.532806],[12.933577,77.532764],[12.9336,77.532687]]},{"t":"r","c":[[12.931283,77.534657],[12.931672,77.534667]]},{"t":"r","c":[[12.931265,77.53497],[12.931852,77.535007]]},{"t":"r","c":[[12.931248,77.535565],[12.931713,77.535605],[12.931828,77.53561],[12.931923,77.535607]]},{"t":"r","c":[[12.939256,77.538518],[12.939206,77.538296],[12.939137,77.538122]]},{"t":"r","c":[[12.937624,77.54289],[12.937612,77.542892],[12.937601,77.54289],[12.937589,77.542883]]},{"t":"t","c":[[12.93827,77.533336],[12.938784,77.533007]]},{"t":"m","c":[[12.936697,77.5312],[12.93706,77.53127],[12.937405,77.531375],[12.93807,77.531696],[12.938188,77.531745]]},{"t":"m","c":[[12.938188,77.531745],[12.938516,77.531879]]},{"t":"t","c":[[12.939875,77.532422],[12.939937,77.532384]]},{"t":"t","c":[[12.934894,77.537477],[12.9348,77.537631]]},{"t":"t","c":[[12.941145,77.531894],[12.940965,77.531905]]},{"t":"r","c":[[12.933623,77.538921],[12.933882,77.538515],[12.934024,77.53827],[12.934114,77.538177]]},{"t":"r","c":[[12.933878,77.535953],[12.934022,77.535959],[12.934257,77.535976]]},{"t":"r","c":[[12.932974,77.535692],[12.933212,77.535756],[12.933363,77.535796]]},{"t":"r","c":[[12.939523,77.536689],[12.939562,77.537054],[12.939562,77.537249]]},{"t":"r","c":[[12.939562,77.537249],[12.939519,77.537346],[12.939508,77.537448],[12.939521,77.537501],[12.939556,77.537543]]},{"t":"t","c":[[12.9348,77.537631],[12.934605,77.537912],[12.933937,77.539038]]},{"t":"t","c":[[12.934718,77.537573],[12.9348,77.537631]]},{"t":"r","c":[[12.932252,77.534309],[12.932093,77.534338]]},{"t":"r","c":[[12.935439,77.533682],[12.935623,77.533707]]},{"t":"r","c":[[12.933212,77.535756],[12.933256,77.535578]]},{"t":"m","c":[[12.938617,77.532523],[12.938455,77.532015]]},{"t":"m","c":[[12.939036,77.532273],[12.938917,77.5323],[12.938823,77.532324],[12.938748,77.532361],[12.938685,77.532416],[12.938642,77.532484],[12.938617,77.532523]]},{"t":"t","c":[[12.934197,77.53823],[12.934712,77.5374],[12.934905,77.537061],[12.93502,77.536858],[12.935225,77.536498],[12.935279,77.536394],[12.935796,77.535404],[12.935949,77.535111],[12.935992,77.53503],[12.936041,77.534936],[12.936163,77.534692],[12.936241,77.534578]]},{"t":"t","c":[[12.939589,77.532769],[12.939454,77.532849]]},{"t":"t","c":[[12.938784,77.533007],[12.939176,77.53276],[12.939405,77.532713]]},{"t":"t","c":[[12.934665,77.53766],[12.934164,77.538509]]},{"t":"t","c":[[12.93364,77.539406],[12.93335,77.539844],[12.933133,77.54015]]},{"t":"t","c":[[12.933721,77.539402],[12.93357,77.539637],[12.933181,77.540165],[12.933023,77.540308]]},{"t":"t","c":[[12.939937,77.532384],[12.941394,77.531509]]},{"t":"t","c":[[12.940965,77.531905],[12.940457,77.532233],[12.939976,77.532531]]},{"t":"t","c":[[12.9362,77.535011],[12.936132,77.534974]]},{"t":"t","c":[[12.936132,77.534974],[12.935836,77.535538],[12.935579,77.536036],[12.935485,77.536212],[12.935365,77.536434],[12.935031,77.537027],[12.934718,77.537573]]},{"t":"r","c":[[12.937031,77.529438],[12.93687,77.529404],[12.936832,77.529394],[12.936803,77.529365],[12.936806,77.529307],[12.936866,77.52906]]},{"t":"r","c":[[12.936654,77.528969],[12.936309,77.528866],[12.935962,77.528763]]},{"t":"r","c":[[12.936675,77.529653],[12.936715,77.52931]]},{"t":"r","c":[[12.939789,77.536014],[12.939552,77.536042]]},{"t":"r","c":[[12.939406,77.535909],[12.939599,77.535889]]},{"t":"t","c":[[12.936519,77.534391],[12.93642,77.534501],[12.936303,77.53464],[12.936197,77.534851]]},{"t":"t","c":[[12.936241,77.534578],[12.936352,77.53444],[12.936458,77.534323]]},{"t":"t","c":[[12.938495,77.533519],[12.93827,77.533336]]},{"t":"t","c":[[12.936197,77.534851],[12.936132,77.534974]]},{"t":"t","c":[[12.936163,77.534692],[12.936093,77.534647]]},{"t":"r","c":[[12.933519,77.542077],[12.933591,77.5421]]},{"t":"r","c":[[12.933591,77.5421],[12.933548,77.542238]]}]};

const state={twin:{},real:{},ues:[],gnbs:[],cfg:{},sba:{}};
let selectedGnbId=null, selectedUeId=null;
let showMap=true;

/* ── OSM coordinate → world simulation coordinate mapping ── */
// OSM bounds → simulation world coordinate space
// Simulation world: UEs in ±500m around gNBs at various (x,y)
// We map the OSM bbox to fit the canvas with simulation coords overlaid
// OSM center → world (0,0); scale so bbox ~= CELL_RADIUS * 2 = ~1000m each side
const OSM_B = OSM_DATA.b;
const OSM_LAT_CTR = (OSM_B.minlat + OSM_B.maxlat) / 2;
const OSM_LON_CTR = (OSM_B.minlon + OSM_B.maxlon) / 2;
// Meters per degree at this latitude
const LAT_M = 111320;
const LON_M = 111320 * Math.cos(OSM_LAT_CTR * Math.PI / 180);
const OSM_HEIGHT_M = (OSM_B.maxlat - OSM_B.minlat) * LAT_M;
const OSM_WIDTH_M  = (OSM_B.maxlon - OSM_B.minlon) * LON_M;

function osmToWorld(lat, lon) {
  // OSM center → world (0,0); lat increases up, lon increases right
  const wx = (lon - OSM_LON_CTR) * LON_M;
  const wy = (lat - OSM_LAT_CTR) * LAT_M;
  return [wx, wy];
}

/* Road style per type */
const ROAD_STYLE = {
  'm': {color:'#334466', width:3.5},  // motorway
  't': {color:'#2a3d55', width:2.5},  // trunk
  's': {color:'#1e3050', width:2},    // secondary
  'e': {color:'#1e3050', width:1.5},  // tertiary (t already used for trunk, so tertiary='e'... wait)
  'r': {color:'#162540', width:1},    // residential
  'p': {color:'#162540', width:0.8},  // primary
};
// Actually t=trunk, r=residential, m=motorway, s=secondary, e=service? let me use first char of each type
// Actual first chars from our data: m=motorway, t=trunk, r=residential, s=secondary, e=tertiary? no...
// From OSM: motorway->m, trunk->t, residential->r, secondary->s, tertiary->e? no, tertiary starts with 't' too
// Our code used htype[0] so: motorway=m, trunk=t, residential=r, secondary=s, tertiary=t (same as trunk!)
// Let me check: we used {'t':'r','c':coords} where 't' is first char. motorway='m', trunk='t', tertiary='t', secondary='s', residential='r'
// So 't' covers both trunk and tertiary - fine for rendering
function roadStyle(t) {
  if(t==='m') return {color:'#2a4a6a',width:3.5,opacity:0.7};
  if(t==='t') return {color:'#1e3a58',width:2.5,opacity:0.6};  // trunk+tertiary
  if(t==='s') return {color:'#1a3048',width:2,opacity:0.55};
  if(t==='r') return {color:'#111e30',width:1,opacity:0.5};    // residential
  return {color:'#101828',width:0.8,opacity:0.4};
}

function fmt(v,dp=1){if(v===undefined||v===null)return'—';return(+v).toFixed(dp);}
function sv(id,v,unit){const el=document.getElementById(id);if(el)el.textContent=v+unit;}
function toast(msg,color='var(--accent)'){
  const t=document.getElementById('toast');
  t.textContent=msg;t.style.borderColor=color;t.style.color=color;
  t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2500);
}
function switchTab(id){
  const ids=['tab-gnb','tab-ue','tab-sys','tab-sample','tab-pcap'];
  document.querySelectorAll('.tab').forEach((t,i)=>t.classList.toggle('active',ids[i]===id));
  document.querySelectorAll('.tab-content').forEach(c=>c.classList.toggle('active',c.id===id));
}
function toggleMap(){
  showMap=!showMap;
  const btn=document.getElementById('map-toggle');
  btn.textContent=showMap?'🗺 MAP ON':'🗺 MAP OFF';
  btn.classList.toggle('on',showMap);
  drawTopology();
}

async function loadPcapStats(){
  try{
    const r=await fetch('/pcap_stats');
    const d=await r.json();
    if(!d.ok)return;
    const ifaces=d.ifaces;
    const COLORS={uu_radio:'var(--accent)',n2_ngap:'var(--yellow)',n3_gtpu:'#cc00ff',
                  f1_du_cu:'#00e5ff',sba_http2:'#ff9800',coap_mmtc:'#e91e63',icmp_ctrl:'#4caf50'};
    const grid=document.getElementById('pcap-stats-grid');
    if(grid)grid.innerHTML=Object.entries(ifaces).map(([n,s])=>`
      <div style="background:var(--panel);border:1px solid var(--border);border-radius:4px;padding:5px 8px">
        <div style="color:${COLORS[n]||'var(--fg)'};font-size:0.62rem;font-weight:600">${n}</div>
        <div style="font-size:0.58rem;color:var(--dim)">buffered: <b style="color:var(--fg)">${s.buffered.toLocaleString()}</b> &nbsp; total: <b style="color:var(--fg)">${s.total.toLocaleString()}</b></div>
      </div>`).join('');
    const dlList=document.getElementById('pcap-dl-list');
    if(dlList)dlList.innerHTML=Object.keys(ifaces).map(n=>`
      <a href="/download_pcap?iface=${n}" download="${n}.pcap"
         style="display:block;padding:6px 10px;background:var(--border);border:1px solid ${COLORS[n]||'var(--border)'};
                color:${COLORS[n]||'var(--fg)'};border-radius:4px;text-decoration:none;font-size:0.62rem;
                cursor:pointer;transition:opacity 0.2s" onmouseover="this.style.opacity=0.75" onmouseout="this.style.opacity=1">
        ⬇ ${n}.pcap &nbsp;<span style="color:var(--dim)">(${ifaces[n].total} pkts)</span>
      </a>`).join('');
  }catch(e){console.error('pcap stats error',e);}
}
setInterval(()=>{
  const t=document.querySelector('.tab.active');
  if(t&&t.textContent.trim()==='PCAP')loadPcapStats();
},3000);
function toggleSection(id){document.getElementById(id).classList.toggle('collapsed');}
setInterval(()=>{const now=new Date();const el=document.getElementById('clock');if(el)el.textContent=now.toTimeString().slice(0,8);},1000);

function nrScs(freqGhz){
  if(freqGhz<=1)return 15000;if(freqGhz<=6)return 30000;
  if(freqGhz<=24)return 60000;if(freqGhz<=52.6)return 120000;return 240000;
}
function numRbFromBw(bwMhz,freqGhz){
  const scs=nrScs(freqGhz);return Math.max(1,Math.floor((bwMhz*1e6)/(scs*12)));
}
function updateGnbRbHint(){
  const fq=parseFloat(document.getElementById('gnb-fq-s').value)||28;
  const bw=parseFloat(document.getElementById('gnb-bw-s').value)||400;
  const scs=nrScs(fq);const rb=numRbFromBw(bw,fq);
  document.getElementById('gnb-rb-hint').textContent=`→ ~${rb} RBs (${scs/1000}kHz SCS)`;
}

/* ── CANVAS ── */
const canvas=document.getElementById('topo');
const ctx=canvas.getContext('2d');
function resizeCanvas(){const wrap=canvas.parentElement;canvas.width=wrap.clientWidth;canvas.height=wrap.clientHeight;drawTopology();}
window.addEventListener('resize',resizeCanvas);
setTimeout(resizeCanvas,100);
let viewScale=1.0,viewOX=0,viewOY=0;

function worldToCanvas(wx,wy){
  const cx=canvas.width/2+viewOX,cy=canvas.height/2+viewOY;
  const sc=Math.min(canvas.width,canvas.height)/2/600*viewScale;
  return[cx+wx*sc,cy-wy*sc];
}
function canvasToWorld(cx,cy){
  const ox=canvas.width/2+viewOX,oy=canvas.height/2+viewOY;
  const sc=Math.min(canvas.width,canvas.height)/2/600*viewScale;
  return[(cx-ox)/sc,-(cy-oy)/sc];
}

/* ── DISTINCT gNB COLORS — all clearly different from eMBB cyan, URLLC yellow, mMTC purple ── */
/* gNBs use warm/neutral colors: bright green, orange, pink, gold, red-orange, white-blue, lime */
const GNB_COLORS=['#00ff9d','#ff8c00','#ff69b4','#ffd700','#ff4500','#87ceeb','#7fff00','#ff6eb4'];
function gnbColor(gid){return GNB_COLORS[gid%GNB_COLORS.length];}

/* ── OSM TILE MAP ── */
// Tile helpers
const OSM_TILE_ZOOM = 16;
const _tileCache = {};

function _latLonToTileXY(lat, lon, zoom) {
  const n = Math.pow(2, zoom);
  const x = Math.floor((lon + 180) / 360 * n);
  const latR = lat * Math.PI / 180;
  const y = Math.floor((1 - Math.log(Math.tan(latR) + 1/Math.cos(latR)) / Math.PI) / 2 * n);
  return {x, y};
}

function _tileTopLeftLatLon(tx, ty, zoom) {
  const n = Math.pow(2, zoom);
  const lon = tx / n * 360 - 180;
  const latR = Math.atan(Math.sinh(Math.PI * (1 - 2 * ty / n)));
  const lat = latR * 180 / Math.PI;
  return {lat, lon};
}

function _getTile(tx, ty, zoom) {
  const key = `${zoom}/${tx}/${ty}`;
  if (_tileCache[key]) return _tileCache[key];
  const img = new Image();
  img.crossOrigin = 'anonymous';
  // Use tile.openstreetmap.org — standard OSM tiles
  img.src = `https://cartodb-basemaps-a.global.ssl.fastly.net/dark_all/${zoom}/${tx}/${ty}.png`;
  img.onload = () => { _tileCache[key].ready = true; drawTopology(); };
  img.onerror = () => { _tileCache[key].err = true; };
  _tileCache[key] = {img, ready: false};
  return _tileCache[key];
}

function drawOsmMap() {
  if (!showMap) return;
  const zoom = OSM_TILE_ZOOM;
  // Compute which tiles cover the current canvas view
  // We need to find the lat/lon of the canvas corners by inverting worldToCanvas
  const [wMinX, wMinY] = canvasToWorld(0, canvas.height);
  const [wMaxX, wMaxY] = canvasToWorld(canvas.width, 0);

  // World → lat/lon
  function worldToLatLon(wx, wy) {
    const lat = OSM_LAT_CTR + wy / LAT_M;
    const lon = OSM_LON_CTR + wx / LON_M;
    return {lat, lon};
  }

  const topLeft  = worldToLatLon(wMinX, wMaxY);
  const botRight = worldToLatLon(wMaxX, wMinY);

  const tMin = _latLonToTileXY(topLeft.lat,  topLeft.lon,  zoom);
  const tMax = _latLonToTileXY(botRight.lat, botRight.lon, zoom);

  // lat/lon of a tile's top-left corner → canvas pixel
  function tileTopLeftCanvas(tx, ty) {
    const {lat, lon} = _tileTopLeftLatLon(tx, ty, zoom);
    const wx = (lon - OSM_LON_CTR) * LON_M;
    const wy = (lat - OSM_LAT_CTR) * LAT_M;
    return worldToCanvas(wx, wy);
  }

  // Pixel size of one tile on the canvas
  function tilePxSize(tx, ty) {
    const [x0, y0] = tileTopLeftCanvas(tx,   ty);
    const [x1, y1] = tileTopLeftCanvas(tx+1, ty+1);
    return {w: x1-x0, h: y1-y0};
  }

  ctx.save();
  for (let ty = tMin.y; ty <= tMax.y; ty++) {
    for (let tx = tMin.x; tx <= tMax.x; tx++) {
      const tile = _getTile(tx, ty, zoom);
      if (!tile.ready) continue;
      const [px, py] = tileTopLeftCanvas(tx, ty);
      const {w, h}   = tilePxSize(tx, ty);
      ctx.drawImage(tile.img, px, py, w, h);
    }
  }
  ctx.restore();
}

function drawTopology(){
  if(!canvas.width)return;
  ctx.clearRect(0,0,canvas.width,canvas.height);
  // Light grey fallback so tiles load gracefully (not black)
  ctx.fillStyle='#1a1a2e';
  ctx.fillRect(0,0,canvas.width,canvas.height);
  const sc=Math.min(canvas.width,canvas.height)/2/600*viewScale;

  // ── 1. OSM TILE MAP (background) ──
  drawOsmMap();

  // ── 2. Coverage circles per gNB ──
  for(const gnb of state.gnbs){
    const[gx,gy]=worldToCanvas(gnb.x,gnb.y);
    const r=(state.cfg.CELL_RADIUS_M||500)*sc;
    const col=gnbColor(gnb.id);
    // Fill
    ctx.beginPath();ctx.arc(gx,gy,r,0,2*Math.PI);
    ctx.fillStyle=col+'0a';ctx.fill();
    // Stroke
    ctx.beginPath();ctx.arc(gx,gy,r,0,2*Math.PI);
    ctx.strokeStyle=col+'50';ctx.lineWidth=1.5;ctx.stroke();
    // Dashed inner circle at 50% radius
    ctx.beginPath();ctx.arc(gx,gy,r*0.5,0,2*Math.PI);
    ctx.setLineDash([4,6]);ctx.strokeStyle=col+'25';ctx.lineWidth=0.8;ctx.stroke();
    ctx.setLineDash([]);
  }

  // ── 3. UE connection lines to serving gNB ──
  const gnbMap={};
  for(const gnb of state.gnbs)gnbMap[gnb.id]=gnb;
  for(const u of state.ues){
    const sg=gnbMap[u.gnbId];if(!sg)continue;
    const[ux,uy]=worldToCanvas(u.x,u.y);
    const[gx,gy]=worldToCanvas(sg.x,sg.y);
    ctx.beginPath();ctx.moveTo(ux,uy);ctx.lineTo(gx,gy);
    ctx.strokeStyle=gnbColor(u.gnbId)+'18';ctx.lineWidth=0.5;ctx.stroke();
  }

  // ── 4. UEs — DISTINCT colors: eMBB=cyan, URLLC=yellow, mMTC=magenta ──
  const SLICE_COLORS={eMBB:'#00e5ff',URLLC:'#ffcc00',mMTC:'#cc00ff'};
  for(const u of state.ues){
    const[px,py]=worldToCanvas(u.x,u.y);
    if(px<-20||px>canvas.width+20||py<-20||py>canvas.height+20)continue;
    const col=u.ho?'#ff3d5a':(SLICE_COLORS[u.slice]||'#888');
    const r=u.id===selectedUeId?5.5:3.5;
    ctx.beginPath();ctx.arc(px,py,r,0,2*Math.PI);
    ctx.fillStyle=col;ctx.fill();
    if(u.id===selectedUeId){
      ctx.beginPath();ctx.arc(px,py,8,0,2*Math.PI);
      ctx.strokeStyle=col+'80';ctx.lineWidth=1.5;ctx.stroke();
    }
  }

  // ── 5. gNBs — bright green tower with ring ──
  for(const gnb of state.gnbs){
    const[px,py]=worldToCanvas(gnb.x,gnb.y);
    const col=gnbColor(gnb.id);
    // Outer glow ring
    ctx.beginPath();ctx.arc(px,py,14,0,2*Math.PI);
    ctx.fillStyle=col+'15';ctx.fill();
    ctx.strokeStyle=col+'60';ctx.lineWidth=1;ctx.stroke();
    // Inner circle
    ctx.beginPath();ctx.arc(px,py,8,0,2*Math.PI);
    ctx.fillStyle=col+'30';ctx.fill();
    ctx.strokeStyle=col;ctx.lineWidth=2;ctx.stroke();
    // Load arc
    if(gnb.e2Load!==undefined&&gnb.e2Load>0){
      ctx.beginPath();ctx.arc(px,py,16,-Math.PI/2,-Math.PI/2+gnb.e2Load*2*Math.PI);
      ctx.strokeStyle='rgba(255,204,0,0.8)';ctx.lineWidth=2.5;ctx.stroke();
    }
    // Label
    ctx.fillStyle=col;ctx.font='bold 9px Share Tech Mono';
    ctx.textAlign='center';ctx.fillText(gnb.label,px,py-20);
    // SINR + UE count beneath
    const ueCount=state.ues.filter(u=>u.gnbId===gnb.id).length;
    const sinrCol=gnb.e2AvgSinr>15?'#00ff9d':gnb.e2AvgSinr>5?'#ffcc00':'#ff3d5a';
    ctx.fillStyle=sinrCol;ctx.font='8px Share Tech Mono';
    ctx.fillText(`${fmt(gnb.e2AvgSinr,1)}dB · ${ueCount}UE`,px,py+28);
    ctx.textAlign='left';
  }
}

/* ── FETCH ── */
async function fetchState(){
  try{
    const r=await fetch('/status',{signal:AbortSignal.timeout(2500)});
    const d=await r.json();
    state.twin=d.twin||{};state.real=d.real||{};
    state.ues=d.ues||[];state.gnbs=d.gnbs||[];
    state.cfg=d.cfg||{};state.sba=d.sba||{};
    renderAll();
    const tU=state.twin.activeUes||0,rU=state.real.activeUes||0;
    const diff=Math.abs(tU-rU);
    const pill=document.getElementById('sync-status');
    if(diff===0){pill.textContent='SYNCED ✓';pill.className='pill green';}
    else if(diff<=3){pill.textContent='SYNC ~';pill.className='pill yellow';}
    else{pill.textContent='DIVERGED';pill.className='pill red';}
    document.getElementById('twin-ues').textContent=`TWIN: ${tU} UEs`;
    document.getElementById('real-ues').textContent=`REAL: ${rU} UEs`;
    const numGnbs=state.gnbs.length;
    const cap=state.twin.systemCapacity||250;
    document.getElementById('gnb-badge').textContent=`gNBs: ${numGnbs}`;
    document.getElementById('cap-badge').textContent=`CAP: ${cap}`;
    document.getElementById('cap-badge').className=tU>cap*0.9?'pill red':'pill orange';
    const w=state.cfg.WEATHER||'NORMAL';
    const wicons={NORMAL:'☀',RAINY:'🌧',WINDY:'💨',FOGGY:'🌫'};
    document.getElementById('weather-badge').textContent=(wicons[w]||'')+' '+w;
    document.getElementById('sched-badge').textContent=state.cfg.SCHEDULER||'PF';
  }catch(e){
    document.getElementById('sync-status').textContent='OFFLINE';
    document.getElementById('sync-status').className='pill red';
  }
}

function renderAll(){
  renderKpis();renderSliceStats();renderCmp();renderGnbs();renderRealPanel();
  renderUeTable();renderSysCfg();drawTopology();renderSampleUes();
}

function kpiCard(label,val,cls,unit=''){
  return`<div class="kpi ${cls}"><div class="kpi-label">${label}</div><div class="kpi-value">${val}<span class="kpi-unit"> ${unit}</span></div></div>`;
}
function renderKpis(){
  const t=state.twin,r=state.real;
  const ud=(t.activeUes||0)-(r.activeUes||0);
  let h='';
  h+=kpiCard('TWIN TPUT',fmt(t.totalTput),'twin','Mbps');
  h+=kpiCard('REAL TPUT',fmt(r.totalTput),'real','Mbps');
  h+=kpiCard('TWIN SINR',fmt(t.avgSinr),'twin','dB');
  h+=kpiCard('REAL SINR',fmt(r.avgSinr),'real','dB');
  h+=kpiCard('TWIN LAT',fmt(t.avgLatency),'twin','ms');
  h+=kpiCard('REAL LAT',fmt(r.avgLatency),'real','ms');
  h+=kpiCard('TWIN BLER',fmt((t.avgBler||0)*100,2),'twin','%');
  h+=kpiCard('REAL BLER',fmt((r.avgBler||0)*100,2),'real','%');
  h+=kpiCard('SPEC EFF',fmt(t.specEffic,3),'twin','b/s/Hz');
  h+=kpiCard('UE Δ',ud>=0?'+'+ud:ud,'delta',Math.abs(ud)<=2?'✓':'⚠');
  document.getElementById('kpi-grid').innerHTML=h;
}
function renderSliceStats(){
  const ss=(state.twin.sliceStats)||{};
  const limits={eMBB:{latMax:30},URLLC:{latMax:4},mMTC:{latMax:1000}};
  let h='';
  for(const[key,label,cls]of[['eMBB','eMBB','embb-row'],['URLLC','URLLC','urllc-row'],['mMTC','mMTC','mmtc-row']]){
    const s=ss[key]||{};const lat=s.avgLatency||0;const limLat=limits[key]?.latMax||9999;
    const latWarn=lat>limLat?`<span class="warn-badge">⚠ >${limLat}ms!</span>`:'';
    h+=`<tr class="${cls}"><td>${label}</td><td>${s.ues||0}</td>
      <td style="color:var(--accent)">${fmt(s.avgTput,2)} Mbps</td>
      <td style="color:${lat>limLat?'var(--red)':'var(--green)'}">${fmt(lat,2)} ms${latWarn}</td></tr>`;
  }
  document.getElementById('slice-tbody').innerHTML=h||'<tr><td colspan="4" style="color:var(--dim);padding:4px 8px">No UEs active</td></tr>';
}
function renderCmp(){
  const t=state.twin,r=state.real;
  const rows=[
    ['Tput Mbps',t.totalTput,r.totalTput,1],['SINR dB',t.avgSinr,r.avgSinr,1],
    ['Lat ms',t.avgLatency,r.avgLatency,1],['BLER %',(t.avgBler||0)*100,(r.avgBler||0)*100,2],
    ['RSRP dBm',t.avgRsrp,r.avgRsrp,1],['UEs',t.activeUes,r.activeUes,0],
    ['eMBB Mbps',t.embbTput,r.embbTput,1],['URLLC Mbps',t.urllcTput,r.urllcTput,1],
    ['mMTC Mbps',t.mmtcTput,r.mmtcTput,1],['Spec Eff',t.specEffic,r.specEffic,3],
    ['RB Util%',(t.rbUtil||0)*100,(r.rbUtil||0)*100,1],
  ];
  let h='';
  for(const[label,tv,rv,dp]of rows){
    const delta=(tv||0)-(rv||0);
    const cls=Math.abs(delta)<0.01?'neu':delta>0?'pos':'neg';
    h+=`<tr><td>${label}</td><td style="color:var(--accent)">${fmt(tv,dp)}</td><td style="color:var(--green)">${fmt(rv,dp)}</td><td class="${cls}">${delta>=0?'+':''}${fmt(delta,dp)}</td></tr>`;
  }
  document.getElementById('cmp-tbody').innerHTML=h;
}

/* ── gNB card render with UE count + IDs ── */
function renderGnbs(){
  let h='';
  for(const gnb of state.gnbs){
    const sel=gnb.id===selectedGnbId;
    const loadPct=((gnb.e2Load||0)*100).toFixed(0);
    const sinrCol=gnb.e2AvgSinr>15?'var(--green)':gnb.e2AvgSinr>5?'var(--yellow)':'var(--red)';
    const col=gnbColor(gnb.id);

    // Gather UEs connected to this gNB
    const connectedUes=state.ues.filter(u=>u.gnbId===gnb.id);
    const ueCount=connectedUes.length;
    const upUes=connectedUes.filter(u=>u.nas==='UP').length;
    const ueIds=connectedUes.map(u=>u.id).sort((a,b)=>a-b);

    // Build slice breakdown
    const embbN=connectedUes.filter(u=>u.slice==='eMBB').length;
    const urllcN=connectedUes.filter(u=>u.slice==='URLLC').length;
    const mmtcN=connectedUes.filter(u=>u.slice==='mMTC').length;

    // Build UE ID list (show up to 40, then ellipsis)
    const MAX_SHOW=40;
    let idsDisplay='';
    if(ueIds.length===0){
      idsDisplay='<span style="color:var(--dim)">No UEs</span>';
    } else {
      const shown=ueIds.slice(0,MAX_SHOW);
      idsDisplay=shown.map(id=>{
        const u=connectedUes.find(x=>x.id===id);
        const slCol=u?{'eMBB':'#00e5ff','URLLC':'#ffcc00','mMTC':'#cc00ff'}[u.slice]||'#888':'#888';
        const nas=u?u.nas:'?';
        return `<span style="color:${slCol};cursor:pointer" onclick="showUeDetail(${id})" title="UE-${id} ${u?u.slice:''} ${nas}">${id}</span>`;
      }).join(' ');
      if(ueIds.length>MAX_SHOW) idsDisplay+=` <span style="color:var(--dim)">+${ueIds.length-MAX_SHOW} more</span>`;
    }

    h+=`<div class="gnb-card${sel?' selected':''}" onclick="selectGnb(${gnb.id})" style="border-left:3px solid ${col}">
      <div class="gnb-card-hdr">
        <div class="gnb-dot" style="background:${col};box-shadow:0 0 5px ${col}"></div>
        <span class="gnb-name" style="color:${col}">${gnb.label}</span>
        <span class="pill blue" style="font-size:0.52rem">ID:${gnb.id}</span>
      </div>
      <div class="gnb-params">
        <span class="gnb-param-label">Position</span><span class="gnb-param-val">(${fmt(gnb.x,0)}, ${fmt(gnb.y,0)}) m</span>
        <span class="gnb-param-label">Tx Power</span><span class="gnb-param-val">${gnb.txPower} dBm</span>
        <span class="gnb-param-label">Freq / BW</span><span class="gnb-param-val">${gnb.freqGhz}GHz/${gnb.bwMhz}MHz</span>
        <span class="gnb-param-label">MIMO</span><span class="gnb-param-val">${gnb.txAnt}T×${gnb.maxLayers||4}L</span>
        <span class="gnb-param-label">E2 SINR</span><span class="gnb-param-val" style="color:${sinrCol}">${fmt(gnb.e2AvgSinr,1)} dB</span>
        <span class="gnb-param-label">E2 Load</span><span class="gnb-param-val">${loadPct}%</span>
      </div>
      <div class="gnb-ue-summary">
        <div class="gnb-ue-summary-hdr">
          Connected UEs
          <span class="gnb-ue-count-badge" style="background:${col}20;color:${col};border:1px solid ${col}60">${ueCount} total · ${upUes} UP</span>
          &nbsp;
          <span style="color:#00e5ff;font-size:0.52rem">eMBB:${embbN}</span>
          <span style="color:#ffcc00;font-size:0.52rem;margin-left:4px">URLLC:${urllcN}</span>
          <span style="color:#cc00ff;font-size:0.52rem;margin-left:4px">mMTC:${mmtcN}</span>
        </div>
        <div class="gnb-ue-ids">${idsDisplay}</div>
      </div>
    </div>`;
  }
  document.getElementById('gnb-list').innerHTML=h||'<div style="padding:8px;color:var(--dim);font-size:0.68rem">No gNBs</div>';
}

function selectGnb(id){
  selectedGnbId=id;
  const gnb=state.gnbs.find(g=>g.id===id);
  if(!gnb)return;
  document.getElementById('sel-gnb-lbl').textContent=gnb.label;
  document.getElementById('gnb-edit-form').style.display='block';
  document.getElementById('edit-gnb-label').value=gnb.label;
  document.getElementById('edit-gnb-x').value=gnb.x;
  document.getElementById('edit-gnb-y').value=gnb.y;
  document.getElementById('edit-gnb-tx').value=gnb.txPower;sv('edit-gnb-tx-v',gnb.txPower,'dBm');
  document.getElementById('edit-gnb-h').value=gnb.height;sv('edit-gnb-h-v',gnb.height,'m');
  document.getElementById('edit-gnb-fq').value=gnb.freqGhz;sv('edit-gnb-fq-v',gnb.freqGhz,'GHz');
  document.getElementById('edit-gnb-bw').value=gnb.bwMhz;sv('edit-gnb-bw-v',gnb.bwMhz,'MHz');
  document.getElementById('edit-gnb-ant').value=gnb.txAnt||64;
  document.getElementById('edit-gnb-layers').value=gnb.maxLayers||4;
  switchTab('tab-gnb');renderGnbs();
}
async function applyGnbEdit(){
  if(selectedGnbId===null){toast('No gNB selected','var(--red)');return;}
  const params={
    label:document.getElementById('edit-gnb-label').value,
    x:parseFloat(document.getElementById('edit-gnb-x').value),
    y:parseFloat(document.getElementById('edit-gnb-y').value),
    tx_power:parseFloat(document.getElementById('edit-gnb-tx').value),
    height:parseFloat(document.getElementById('edit-gnb-h').value),
    freq_ghz:parseFloat(document.getElementById('edit-gnb-fq').value),
    bw_mhz:parseFloat(document.getElementById('edit-gnb-bw').value),
    tx_ant:parseInt(document.getElementById('edit-gnb-ant').value),
    max_layers:parseInt(document.getElementById('edit-gnb-layers').value),
  };
  const r=await fetch('/update_gnb',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:selectedGnbId,...params})});
  const d=await r.json();
  toast(d.ok?`gNB ${selectedGnbId} updated ✓`:'Error: '+d.error,d.ok?'var(--green)':'var(--red)');
}
async function deleteGnb(){
  if(selectedGnbId===null){toast('No gNB selected','var(--red)');return;}
  if(!confirm(`Delete gNB ${selectedGnbId}?`))return;
  const r=await fetch('/delete_gnb',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:selectedGnbId})});
  const d=await r.json();
  if(d.ok){selectedGnbId=null;document.getElementById('gnb-edit-form').style.display='none';}
  toast(d.ok?'gNB deleted':'Error','var(--red)');
}
function renderRealPanel(){
  const r=state.real;
  const rows=[
    ['UEs',r.activeUes||0],['Connected',r.connectedUes||0],
    ['Tput',fmt(r.totalTput)+' Mbps'],['SINR',fmt(r.avgSinr)+' dB'],
    ['Latency',fmt(r.avgLatency)+' ms'],['BLER',fmt((r.avgBler||0)*100,2)+'%'],
    ['eMBB',fmt(r.embbTput)+' Mbps'],['URLLC',fmt(r.urllcTput)+' Mbps'],
    ['mMTC',fmt(r.mmtcTput)+' Mbps'],['Spec Eff',fmt(r.specEffic,3)+' b/s/Hz'],
    ['RB Util',fmt((r.rbUtil||0)*100,1)+'%'],
  ];
  let h='';
  for(const[l,v]of rows)
    h+=`<div class="real-row"><span class="real-label">${l}</span><span class="real-val">${v}</span></div>`;
  document.getElementById('real-panel').innerHTML=`<div style="padding:3px 12px">${h}</div>`;
}
function renderSysCfg(){
  const c=state.cfg;if(!c||Object.keys(c).length===0)return;
  const show=[
    ['GNB_TX_DBM','Tx Power','dBm'],['FREQ_GHZ','Frequency','GHz'],['BW_MHZ','Bandwidth','MHz'],
    ['NOISE_FIG_DB','Noise Fig','dB'],['NUM_RB','Num RBs',''],
    ['RB_EMBB','RB eMBB',''],['RB_URLLC','RB URLLC',''],['RB_MMTC','RB mMTC',''],
    ['GNB_TX_ANT','TX Ant',''],['MAX_LAYERS','Max Lyr',''],
    ['WEATHER','Weather',''],['SCHEDULER','Scheduler',''],
  ];
  let h='<div class="params-grid">';
  for(const[k,lbl,unit]of show){
    const v=c[k];if(v===undefined)continue;
    h+=`<span class="p-label">${lbl}</span><span style="color:var(--accent)">${v}${unit}</span>`;
  }
  h+='</div>';
  document.getElementById('sys-cfg').innerHTML=h;
  const el2=document.getElementById('sys-cfg-tab');if(el2)el2.innerHTML=h;
}
function renderUeTable(){
  const ues=state.ues.slice(0,120);
  document.getElementById('ue-count-badge').textContent=state.ues.length;
  let h='';
  for(const u of ues){
    const sl=u.slice.toLowerCase();
    // Use distinct class names mapped to the new palette
    const scls=sl==='embb'?'slice-embb':sl==='urllc'?'slice-urllc':'slice-mmtc';
    const ho=u.ho?`<span class="ue-ho">HO</span>`:'';
    const sel=u.id===selectedUeId;
    const gcol=gnbColor(u.gnbId);
    h+=`<tr class="${sel?'selected':''}" onclick="showUeDetail(${u.id})">
      <td>${u.id}</td><td class="${scls}">${u.slice}</td><td>${u.nas}</td>
      <td>${fmt(u.sinr,1)}</td><td>${u.cqi}</td><td>${u.mod}</td><td>${u.layers}</td>
      <td>${fmt(u.tput,1)}</td><td>${fmt(u.latency,1)}</td><td>${fmt(u.bler,3)}</td>
      <td>${fmt(u.rsrp,1)}</td><td>${fmt(u.rsrq,2)}</td><td>${fmt(u.doppler,0)}</td>
      <td>${fmt(u.speed,1)}</td><td>${u.allocRb}</td>
      <td style="color:${gcol}">${u.gnbId}</td>
      <td>${ho}</td>
      <td style="color:var(--dim);font-size:0.53rem">${u.pduIp||'—'}</td>
      <td><button class="danger sm" style="padding:1px 4px;font-size:0.53rem" onclick="event.stopPropagation();removeUe(${u.id})">×</button></td>
    </tr>`;
  }
  document.getElementById('ue-tbody').innerHTML=h;
}
function pickSamples(ues,slice,n=5){
  const pool=ues.filter(u=>u.slice===slice||u.slice.toLowerCase()===slice.toLowerCase());
  if(pool.length<=n)return pool;
  const arr=[...pool];
  for(let i=arr.length-1;i>0;i--){const j=Math.floor(Math.random()*(i+1));[arr[i],arr[j]]=[arr[j],arr[i]];}
  return arr.slice(0,n);
}
function sliceColor(slice){
  if(slice==='eMBB'||slice==='EMBB')return'#00e5ff';
  if(slice==='URLLC')return'#ffcc00';
  return'#cc00ff';  // mMTC — distinct magenta/purple
}
function renderSampleUes(){
  const allUes=state.ues;
  const slicePairs=[['eMBB','sample-embb'],['URLLC','sample-urllc'],['mMTC','sample-mmtc']];
  const latLimit={eMBB:30,URLLC:4,mMTC:1000};
  for(const[sl,domId]of slicePairs){
    const samples=pickSamples(allUes,sl);
    if(!samples.length){
      document.getElementById(domId).innerHTML=`<div style="color:var(--dim);font-size:0.6rem;padding:4px">No ${sl} UEs active</div>`;
      continue;
    }
    let h='';
    for(const u of samples){
      const col=sliceColor(u.slice);
      const lat=u.latency||0;const lim=latLimit[sl]||999;
      const latCol=lat>lim?'var(--red)':'var(--green)';
      const gCol=gnbColor(u.gnbId);
      h+=`<div class="sample-ue">
        <div class="sample-ue-hdr">
          <span class="sample-ue-id" style="color:${col}">UE-${u.id}</span>
          <span class="pill" style="border-color:${col};color:${col};padding:1px 5px;font-size:0.48rem">${u.slice}</span>
          <span style="font-size:0.52rem;color:${gCol};margin-left:auto">→gNB-${u.gnbId}</span>
        </div>
        <div class="sample-ue-metrics">
          <span class="sm-label">SINR</span><span class="sm-val">${fmt(u.sinr,1)} dB</span><span></span>
          <span class="sm-label">CQI</span><span class="sm-val">${u.cqi}</span>
          <span class="sm-label">Mod</span><span class="sm-val">${u.mod}</span>
          <span class="sm-label">Tput</span><span class="sm-val">${fmt(u.tput,2)} Mbps</span><span></span>
          <span class="sm-label">Lat</span><span class="sm-val" style="color:${latCol}">${fmt(u.latency,2)} ms</span><span></span>
          <span class="sm-label">BLER</span><span class="sm-val">${fmt(u.bler,3)}</span>
          <span class="sm-label">RSRP</span><span class="sm-val">${fmt(u.rsrp,1)}</span>
          <span class="sm-label">RBs</span><span class="sm-val">${u.allocRb}</span>
          <span class="sm-label">Dist</span><span class="sm-val">${fmt(u.dist,0)} m</span>
          <span class="sm-label">NAS</span><span class="sm-val">${u.nas}</span><span></span>
        </div>
      </div>`;
    }
    document.getElementById(domId).innerHTML=h;
  }
}
function showUeDetail(id){
  selectedUeId=id;
  const u=state.ues.find(x=>x.id===id);
  if(!u){closeDetail();return;}
  const fields=[
    ['ID',u.id],['Slice',u.slice],['NAS',u.nas],['gNB',u.gnbId],
    ['SINR',fmt(u.sinr,2)+' dB'],['CQI',u.cqi],['Mod',u.mod],['Layers',u.layers],
    ['RSRP',fmt(u.rsrp,1)+' dBm'],['RSRQ',fmt(u.rsrq,2)+' dB'],
    ['BLER',fmt(u.bler,4)],['Tput',fmt(u.tput,2)+' Mbps'],
    ['Latency',fmt(u.latency,2)+' ms'],['Doppler',fmt(u.doppler,0)+' Hz'],
    ['Speed',fmt(u.speed,2)+' m/s'],['RBs',u.allocRb],
    ['Pos','('+fmt(u.x,0)+','+fmt(u.y,0)+') m'],['Dist',fmt(u.dist,1)+' m to gNB'],
    ['IP',u.pduIp||'—'],['HO',u.ho?'YES':'NO'],
  ];
  let h='';
  for(const[l,v]of fields)
    h+=`<span style="color:var(--dim)">${l}</span><span>${v}</span>`;
  document.getElementById('det-grid').innerHTML=h;
  document.getElementById('ue-detail').style.display='block';
  document.getElementById('move-id').value=id;
  document.getElementById('move-x').value=fmt(u.x,0);
  document.getElementById('move-y').value=fmt(u.y,0);
}
function closeDetail(){document.getElementById('ue-detail').style.display='none';selectedUeId=null;}

/* ── API ACTIONS ── */
async function addGnb(){
  const body={
    x:parseFloat(document.getElementById('gnb-x').value)||0,
    y:parseFloat(document.getElementById('gnb-y').value)||0,
    txPower:parseFloat(document.getElementById('gnb-tx-s').value)||46,
    label:document.getElementById('gnb-label').value||'gNB-Twin',
    height:parseFloat(document.getElementById('gnb-h-s').value)||30,
    freqGhz:parseFloat(document.getElementById('gnb-fq-s').value)||28,
    bwMhz:parseFloat(document.getElementById('gnb-bw-s').value)||400,
    txAnt:parseInt(document.getElementById('gnb-ant').value)||64,
    maxLayers:parseInt(document.getElementById('gnb-layers').value)||4,
    rbEmbb:parseInt(document.getElementById('gnb-rbe').value)||216,
    rbUrllc:parseInt(document.getElementById('gnb-rbu').value)||27,
    rbMmtc:parseInt(document.getElementById('gnb-rbm').value)||27,
  };
  const r=await fetch('/add_gnb',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const d=await r.json();
  const rb=numRbFromBw(body.bwMhz,body.freqGhz);
  toast(d.ok?`gNB "${body.label}" added (ID ${d.gnbId}) — ${rb} RBs @ ${nrScs(body.freqGhz)/1000}kHz SCS`:'Error',d.ok?'var(--green)':'var(--red)');
}
async function addUes(){
  const xv=document.getElementById('ue-x').value;const yv=document.getElementById('ue-y').value;
  const body={
    count:parseInt(document.getElementById('ue-count').value)||1,
    slice:document.getElementById('ue-slice').value,
    speed:parseFloat(document.getElementById('ue-speed-s').value),
  };
  if(xv!=='')body.x=parseFloat(xv);if(yv!=='')body.y=parseFloat(yv);
  const r=await fetch('/add_ue',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const d=await r.json();
  toast(d.ok?`Added ${d.added} UEs`:'Error',d.ok?'var(--accent)':'var(--red)');
}
async function moveUe(){
  const id=parseInt(document.getElementById('move-id').value)||0;
  const x=parseFloat(document.getElementById('move-x').value)||0;
  const y=parseFloat(document.getElementById('move-y').value)||0;
  const r=await fetch('/move_ue',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id,x,y})});
  const d=await r.json();
  toast(d.ok?`UE ${id} moved`:'Error: '+d.error,d.ok?'var(--accent2)':'var(--red)');
}
async function removeUe(id){
  const r=await fetch('/remove_ue',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id})});
  const d=await r.json();
  if(d.ok&&selectedUeId===id)closeDetail();
  toast(d.ok?`UE ${id} removed`:'Not found',d.ok?'var(--red)':'var(--dim)');
}
async function removeUeById(){await removeUe(parseInt(document.getElementById('move-id').value)||0);}
async function bulkRemove(){
  const n=parseInt(document.getElementById('bulk-rm').value)||10;
  const active=state.ues.slice(0,n).map(u=>u.id);
  for(const id of active)await fetch('/remove_ue',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id})});
  toast(`Removed ${active.length} UEs`,'var(--red)');
}
async function applyRbAlloc(){
  const embb=parseInt(document.getElementById('r-embb').value)||216;
  const urllc=parseInt(document.getElementById('r-urllc').value)||27;
  const mmtc=parseInt(document.getElementById('r-mmtc').value)||27;
  await fetch('/tweak_rb',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({gid:-1,embb,urllc,mmtc})});
  toast('RB allocation updated on all gNBs 📊');
}
async function applyScheduler(){
  const sched=document.getElementById('r-sched').value;
  await fetch('/set_param',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:'SCHEDULER',value:sched})});
  toast('Scheduler: '+sched);
}
async function setWeather(w){
  document.querySelectorAll('.weather-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById('w-'+w).classList.add('active');
  const descs={
    NORMAL:'No propagation impairments.',RAINY:'Rain attenuation +8 dB, 40% higher shadow fading.',
    WINDY:'Light attenuation +2 dB, faster UE mobility (1.6×).',FOGGY:'Heavy fog +12 dB loss, severe fading.',
  };
  document.getElementById('weather-desc').textContent=descs[w]||'';
  await fetch('/set_param',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:'WEATHER',value:w})});
  toast('Weather: '+w,'var(--orange)');
}
async function applyCellGeom(){
  const params=[
    ['CELL_RADIUS_M',parseFloat(document.getElementById('r-cr').value)],
    ['ISD_M',parseFloat(document.getElementById('r-isd').value)],
  ];
  for(const[k,v]of params)
    await fetch('/set_param',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:k,value:v})});
  toast('Cell geometry applied 📐');
}
async function resetTwin(){
  if(!confirm('Reset Digital Twin to default state?'))return;
  await fetch('/reset_twin',{method:'POST',headers:{'Content-Type':'application/json'}});
  toast('Twin reset ↺','var(--red)');
}

/* ── CANVAS INTERACTIONS ── */
canvas.addEventListener('click',async(e)=>{
  const rect=canvas.getBoundingClientRect();
  const[wx,wy]=canvasToWorld(e.clientX-rect.left,e.clientY-rect.top);
  for(const u of state.ues){
    const dx=wx-u.x,dy=wy-u.y;
    if(Math.sqrt(dx*dx+dy*dy)<18){showUeDetail(u.id);return;}
  }
  const slice=document.getElementById('ue-slice').value;
  const speed=parseFloat(document.getElementById('ue-speed-s').value)||2;
  await fetch('/add_ue',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({count:1,slice,x:wx,y:wy,speed})});
  toast(`UE placed at (${wx.toFixed(0)},${wy.toFixed(0)}) — ${slice.toUpperCase()}`);
});
canvas.addEventListener('contextmenu',async(e)=>{
  e.preventDefault();
  const rect=canvas.getBoundingClientRect();
  const[wx,wy]=canvasToWorld(e.clientX-rect.left,e.clientY-rect.top);
  const tx=parseFloat(document.getElementById('gnb-tx-s').value)||46;
  const h=parseFloat(document.getElementById('gnb-h-s').value)||30;
  const fq=parseFloat(document.getElementById('gnb-fq-s').value)||28;
  const bw=parseFloat(document.getElementById('gnb-bw-s').value)||400;
  const ant=parseInt(document.getElementById('gnb-ant').value)||64;
  const layers=parseInt(document.getElementById('gnb-layers').value)||4;
  const rbe=parseInt(document.getElementById('gnb-rbe').value)||216;
  const rbu=parseInt(document.getElementById('gnb-rbu').value)||27;
  const rbm=parseInt(document.getElementById('gnb-rbm').value)||27;
  const lbl=document.getElementById('gnb-label').value||'gNB-Twin';
  const r=await fetch('/add_gnb',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({x:wx,y:wy,txPower:tx,label:lbl,height:h,freqGhz:fq,bwMhz:bw,
      txAnt:ant,maxLayers:layers,rbEmbb:rbe,rbUrllc:rbu,rbMmtc:rbm})});
  const d=await r.json();
  const rb=numRbFromBw(bw,fq);
  toast(d.ok?`gNB "${lbl}" placed at (${wx.toFixed(0)},${wy.toFixed(0)}) — ${rb} RBs`:'Error','var(--green)');
});

/* ── START ── */
updateGnbRbHint();
fetchState();
setInterval(fetchState,1000);
</script>
</body>
</html>"""

                                                               
              
                                                               
_twin: TwinSim = None

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin","*")
        self.send_header("Access-Control-Allow-Methods","GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers","Content-Type")

    def _json(self, obj, code=200):
        body=json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self._cors()
        self.send_header("Content-Length",str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _html(self, html):
        body=html.encode()
        self.send_response(200)
        self.send_header("Content-Type","text/html; charset=utf-8")
        self.send_header("Content-Length",str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200); self._cors()
        self.send_header("Content-Length","0"); self.end_headers()

    def do_GET(self):
        path=urlparse(self.path).path
        if path in ("/","/index.html"):
            self._html(DASHBOARD_HTML)
        elif path=="/status":
            self._json(_twin.get_status())
        elif path=="/pcap_stats":
            stats = pcap_engine.stats()
            self._json({"ok": True, "ifaces": stats,
                        "pcap_dir": os.path.abspath(PCAP_DIR)})
        elif path=="/download_pcap":
            qs   = parse_qs(urlparse(self.path).query)
            iface= qs.get("iface",["uu_radio"])[0]
            data = pcap_engine.download(iface)
            if not data:
                self._json({"error": f"unknown interface: {iface}"}, 404)
                return
            fname = f"{iface}.pcap"
            self.send_response(200)
            self.send_header("Content-Type", "application/vnd.tcpdump.pcap")
            self.send_header("Content-Disposition", f'attachment; filename="{fname}"')
            self.send_header("Content-Length", str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)
        else:
            self._json({"error":"not found"},404)

    def do_POST(self):
        path=urlparse(self.path).path
        length=int(self.headers.get("Content-Length",0))
        try:
            body=json.loads(self.rfile.read(length) or b"{}") if length else {}
        except Exception:
            body={}

        if path=="/add_gnb":
            gid=_twin.add_gnb(
                float(body.get("x",0)), float(body.get("y",0)),
                float(body.get("txPower",CFG["GNB_TX_DBM"])),
                body.get("label",""),
                float(body.get("height",CFG["GNB_HEIGHT_M"])),
                float(body.get("freqGhz",CFG["FREQ_GHZ"])),
                float(body.get("bwMhz",CFG["BW_MHZ"])),
                int(body.get("txAnt",CFG["GNB_TX_ANT"])),
                None,                                    
                int(body.get("rbEmbb",CFG["RB_EMBB"])) if "rbEmbb" in body else None,
                int(body.get("rbUrllc",CFG["RB_URLLC"])) if "rbUrllc" in body else None,
                int(body.get("rbMmtc",CFG["RB_MMTC"])) if "rbMmtc" in body else None,
                int(body.get("maxLayers",CFG["MAX_LAYERS"])),
            )
            self._json({"ok":True,"gnbId":gid})

        elif path=="/update_gnb":
            gid=int(body.get("id",0))
            params={}
            for k,attr in {"label":"label","x":"x","y":"y","tx_power":"tx_power",
                           "height":"height","freq_ghz":"freq_ghz","bw_mhz":"bw_mhz",
                           "tx_ant":"tx_ant","max_layers":"max_layers"}.items():
                if k in body: params[attr]=body[k]
            ok=_twin.update_gnb(gid,params)
            self._json({"ok":ok} if ok else {"ok":False,"error":"gNB not found"})

        elif path=="/delete_gnb":
            ok=_twin.delete_gnb(int(body.get("id",0)))
            self._json({"ok":ok})

        elif path=="/add_ue":
            fx=body.get("x"); fy=body.get("y")
            fs=body.get("speed"); fh=body.get("heading")
            added=_twin.add_ue(
                int(body.get("count",1)), body.get("slice","embb"),
                float(fx) if fx is not None else None,
                float(fy) if fy is not None else None,
                float(fs) if fs is not None else None,
                float(fh) if fh is not None else None,
            )
            self._json({"ok":True,"added":added})

        elif path=="/move_ue":
            ok=_twin.move_ue(int(body.get("id",0)),
                             float(body.get("x",0)),float(body.get("y",0)))
            self._json({"ok":ok} if ok else {"ok":False,"error":"UE not found"})

        elif path=="/remove_ue":
            ok=_twin.remove_ue(int(body.get("id",0)))
            self._json({"ok":ok})

        elif path=="/set_param":
            ok=_twin.set_param(body.get("key",""), body.get("value",0))
            self._json({"ok":ok} if ok else {"ok":False,"error":"unknown param"})

        elif path=="/set_gnb_param":
            ok=_twin.set_gnb_param(
                int(body.get("gid",0)), body.get("key",""), body.get("value",0))
            self._json({"ok":ok})

        elif path=="/tweak_rb":
            _twin.tweak_rb(
                int(body.get("gid",-1)),
                int(body.get("embb",CFG["RB_EMBB"])),
                int(body.get("urllc",CFG["RB_URLLC"])),
                int(body.get("mmtc",CFG["RB_MMTC"])),
            )
            self._json({"ok":True})

        elif path=="/reset_twin":
            _twin.reset()
            self._json({"ok":True})

        else:
            self._json({"error":"not found"},404)

                                                               
      
                                                               
def main():
    global _twin
    print("\033[1;35m╔══════════════════════════════════════════════════════════════╗\033[0m")
    print("\033[1;35m║  5G NR Digital Twin — NS3 Engine v12        ║\033[0m")
    print("\033[1;35m║  NS3 Engine: NsLteAmc · NsThreeGpp · NsLteSpectrumPhy · NsNrPhy  ║\033[0m")
    print("\033[1;35m║  NWDAF · NEF · AMF/SMF · PCF · Near-RT RIC · UPF            ║\033[0m")
    print("\033[1;35m╚══════════════════════════════════════════════════════════════╝\033[0m")
    print()
    running=[True]
    print("[1/3] Starting Real Network simulator …")
    real_sim=RealisticSim()
    threading.Thread(target=real_sim.run,args=(running,),daemon=True).start()
    print("[2/3] Starting Digital Twin (multi-gNB, math-coherent) …")
    _twin=TwinSim(real_sim)
    threading.Thread(target=_twin.run,args=(running,),daemon=True).start()
    time.sleep(0.5)
    print(f"[3/3] Dashboard → \033[1;36mhttp://localhost:{HTTP_PORT}\033[0m")
    print()
    print("  Left-click canvas  → place UE")
    print("  Right-click canvas → place gNB (uses gNB-tab params)")
    print(f"  PCAP files        → {os.path.abspath(PCAP_DIR)}/")
    print("  PCAP stats API    → http://localhost:9095/pcap_stats")
    print("  Download PCAP     → http://localhost:9095/download_pcap?iface=uu_radio")
    print("  Ctrl+C to stop")
    print()
    server=HTTPServer(("0.0.0.0",HTTP_PORT),Handler)
    server.timeout=0.5
    def shutdown(sig,frame):
        running[0]=False
        print("\n\033[1;33mStopping…\033[0m")
        server.server_close(); sys.exit(0)
    signal.signal(signal.SIGINT,shutdown)
    signal.signal(signal.SIGTERM,shutdown)
    try:
        while running[0]:
            server.handle_request()
            _ns3_drain()   # execute any pending NS-3 calls posted by background threads
    except KeyboardInterrupt:
        shutdown(None,None)

if __name__=="__main__":
    main()
