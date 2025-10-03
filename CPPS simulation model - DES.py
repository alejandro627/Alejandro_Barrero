







# CPPS Wooden Box Line - First Draft DES (SimPy)
# Time unit: seconds
# Horizon: one 8-hour shift (28_800 s)

import simpy  ####for DES - discrete-event simulation
import random  #### for random number generation - stochastic behavior to simulations
import statistics ####  or basic statistical operations
from collections import defaultdict, deque ### to create default values automatically (in the meantime) #revisar

RANDOM_SEED = 42 ### This sets a fixed seed for the random number generator, the number can be changed
SHIFT_SECONDS = 8 * 60 * 60  # 8 hours = 28,800 s - This defines the duration of one work shift in seconds

def u(a, b):
    return random.uniform(a, b) #### simple helper to draw uniform random times between a and b seconds

######## Parameters from case study

#### CNC (Manufacturing)
CNC_LOAD_UNLOAD = (15, 25)        # s
CNC_CUT = (60, 120)               # s (path-length dependent)
CNC_TOOL_CHANGE_EVERY = 30        # panels
CNC_TOOL_CHANGE_TIME = 60         # s
# For line-level sense-check: ~120 s/panel on average -> ~30 UPH

# Robot actions
ROBOT_PNP_ANG = (6, 10)           # pick & place ONE angular element
ROBOT_PREP_HOLD = (4, 8)          # prepare posture to hold side wall
ROBOT_RETURN_HOME = (3, 6)        # go back to initial position

# Human actions
HUMAN_PLACE_BASE = (6, 10)        # H1: place base wall
HUMAN_INS_ANG_AND_BOTTOM = (8, 14)# H2: insert 1st angular + bottom wall
HUMAN_INS_ANG_AND_SIDE1 = (10, 20)# H3: insert 2nd angular + 1st side wall
HUMAN_SCREW_SIDE = (15, 30)       # H4/H7: insert screws to fix side wall(s)
HUMAN_MARKER_ROTATE = (6, 12)     # H5: insert marker & rotate the box
HUMAN_INS_ANG34_AND_SIDE2 = (12, 22) # H6: insert 3rd&4th angular + 2nd side wall
HUMAN_PLACE_ON_CONV = (4, 8)      # H8: place assembled box on conveyor

# NOTE: Adhesive is NOT used (per your process sheet)

# Envelope for the on-station time (sanity clamp; not strict)
ASM_STATION_ENVELOPE = (45, 120)  # seconds


### QC & Packaging 
QC_INSPECTION_RATE = (3, 4)       # units/min for visual (convert below)
PACK_TIME_MIN = 60                # 1 min
PACK_TIME_MAX = 180               # 3 min
QC_PACK_TOTAL_TARGET = 4 * 60     # "≈ 4 minutes per unit" guide

#### Storage (Put-away)
PUTAWAY_TIME = (30, 90)           # 0.5–1.5 min in seconds

#### Disruptions (simple first draft)
RAW_MATERIAL_PAUSE_EVERY = 1200   # mean every 20 min (Poisson) -> supplier hiccup
RAW_MATERIAL_PAUSE_DUR = (30, 120)  # 0.5–2 min pause

CNC_MTBF = 3600                   # mean time between failure ~1h
CNC_MTTR = (120, 300)             # repair time 2–5 min

ASM_MTBF = 5400                   # failures less frequent
ASM_MTTR = (120, 300)

OP_ERROR_P_ASM = 0.02             # 2% assembly error -> rework
OP_ERROR_P_QC  = 0.015            # 1.5% QC fail -> rework (send back to Assembly)

#### Demand / target (not enforced yet; source is push)
TARGET_UPH = 30                   # both CNC and QC+packaging operate around 30 UPH → they are the bottlenecks
TARGET_PER_SHIFT = TARGET_UPH * 8 # ~240 units -  30 boxes/hour×8 hours=240 boxes/shift

def opcua_write(tag, value):  # OPC UA stub 
    print(f"[OPC UA] {tag} = {value} at {env.now:.1f}s")


####### Data collection structures

class Stats:         ### to collect and store performance data (KPIs) during the run
    def __init__(self):        ###the __init__ constructor
        self.enter_time = {}          # dictionary  ##entity_id -> time entered system  ## Useful for calculating cycle time later.
        self.cycle_times = []         # completed entities' system cycle time  ##A list storing how long each completed box took from entry to exit ###This is where system cycle times are recorded
        self.completed = 0             #counter for total boxes that finish the system (successful throughput)
        self.reworked = 0             #Counter for how many boxes required rework at Assembly (due to operator/cobot errors).
        self.qc_rejects = 0           #Counter for how many boxes failed QC and had to be sent back for re-assembly
        self.wip_samples = []         # (t, WIP)  ##A list of tuples (time, WIP) storing snapshots of Work-In-Progress (number of boxes in the system at given times)
        self.util_busy = defaultdict(float)   # station_name -> busy_time  #A dictionary storing how long each station was busy
        self.util_down = defaultdict(float)   # station_name -> downtime  ##Similar dictionary for downtime per station (when the station was broken/under repair).
        self.last_update = defaultdict(lambda: 0.0)   ###Tracks the last time a station state was updated (busy, idle, down)
        self.in_system = 0             ##Counter of how many boxes are currently inside the system (real-time WIP)

    def mark_enter(self, env, eid):           #Called when a box enters the system.
        self.enter_time[eid] = env.now        ###Records entry time in enter_time and increases in_system by 1
        self.in_system += 1

    def mark_complete(self, env, eid):           ###Called when a box exits the system.
        t0 = self.enter_time.pop(eid, env.now)
        self.cycle_times.append(env.now - t0)      ##Looks up the entry time, calculates cycle time (exit - entry), and stores it.
        self.completed += 1                      ##Increases completed count, and decreases in_system by 1
        self.in_system -= 1

    def sample_wip(self, env):                                ##Takes a snapshot of the current WIP (in_system) at the current simulation time (env.now).
        self.wip_samples.append((env.now, self.in_system))     ##Useful for plotting WIP trends.

    def add_busy(self, name, start, end):                ##Adds to the total busy time for a station between start and end.
        self.util_busy[name] += max(0, end - start)       ##Helps calculate utilization later.

    def add_down(self, name, start, end):               ###Adds to the total downtime for a station between start and end.
        self.util_down[name] += max(0, end - start)

stats = Stats()             ### Creates one global Stats object that the whole simulation will use to log performance data

# Station abstraction
class Station:
    """
    A single-capacity station with:
    - processing_time() -> draw in seconds
    - optional breakdown process with MTBF/MTTR
    - SimPy Resource to seize/release
    """
    def __init__(self, env, name, processing_time_fn, mtbf=None, mttr_range=None):
        self.env = env             ### Keeps a reference to the SimPy environment (simulation clock).
        self.name = name           ## The station’s label (“CNC”, “Assembly”, “QC”, etc.).
        self.res = simpy.Resource(env, capacity=1)   ## Creates a SimPy Resource with 1 capacity (only one box at a time). This enforces the idea that a CNC or workstation can process only one unit at once.
        self.ptime = processing_time_fn    ##Function that generates a random process time for this station.
        self.mtbf = mtbf           ## it's for Mean Time Between Failures, which is the average time before machine fails
        self.mttr = mttr_range     ## Mean Time To Repair, which is the random repair duration when it breaks
        self.down = False        ## Tracks whether the station is currently broken.
        # bookkeeping for utilization 
        self._busy_start = None     ##Used to mark when a process started (for utilization calculation).
        if self.mtbf:          ## If the station has failures, start a parallel SimPy process _breakdowns() that keeps generating random breakdown events
            self.breakdowns_proc = env.process(self._breakdowns())
    def _breakdowns(self):  ###Generate random breakdowns using exponential MTBF and uniform MTTR
        while True:       # Wait to next failure (if processing, it will interrupt via flag)
            ttf = random.expovariate(1 / self.mtbf)   ## Draw a random time-to-failure from an exponential distribution; with MTBF=3600, the CNC fails about once per hour on average
            yield self.env.timeout(ttf)  ## Wait until that failure occurs.
            down_start = self.env.now    ##            # Enter down state
            self.down = True

            if self._busy_start is not None:    # If was busy, close that busy interval now (conservative)
                stats.add_busy(self.name, self._busy_start, self.env.now)
                self._busy_start = None
            

            rdur = u(*self.mttr)    # Repair duration  ## a uniform repair time between MTTR min and max
            yield self.env.timeout(rdur)
            self.down = False
            stats.add_down(self.name, down_start, self.env.now)  ## Record the downtime period
    def process(self, entity_id):
        """
        Seize capacity, wait if down, process, release.
        Returns actual processing time experienced (for utilization).
        """
        with self.res.request() as req:   # Wait for capacity  ## wait until this station is free
            yield req    
            while self.down:   # If station is down, wait until available; the entity just waits until the station is repaired
                yield self.env.timeout(1)

            # Start processing
            self._busy_start = self.env.now  ##Record the start time
            p = self.ptime()     ## Generate processing time
            yield self.env.timeout(p)   ## Hold for that duration
            # Finish processing
            if self._busy_start is not None:
                stats.add_busy(self.name, self._busy_start, self.env.now)
                self._busy_start = None


# Processing-time functions for each station
# These functions tell the simulation how long each station takes for a given job.

# CNC with tool-change every k panels. We keep a simple counter.
class CNCTime:        ## This defines a class to keep track of how many panels have been processed.
    def __init__(self, change_every, change_time): ## Example: every 30 panels, add 60 seconds for tool change.
        self.count = 0
        self.change_every = change_every
        self.change_time = change_time
    def __call__(self):
        # base: load + cut + unload
        base = u(*CNC_LOAD_UNLOAD) + u(*CNC_CUT) + u(*CNC_LOAD_UNLOAD)
        self.count += 1
        if self.count % self.change_every == 0:
            base += self.change_time
        return base
cnc_time = CNCTime(CNC_TOOL_CHANGE_EVERY, CNC_TOOL_CHANGE_TIME)

# ========== Assembly Cell ==========
def _parallel(a, b): return max(a, b)  # Helper: if robot & human work in parallel, duration = the longer of the two

class AssemblyCell:
    def __init__(self, env, mtbf=None, mttr_range=None):
        self.env = env                                # Reference to SimPy simulation environment (the clock)
        self.robot = simpy.Resource(env, capacity=1)  # Resource representing the robot (can handle 1 task at a time)
        self.human = simpy.Resource(env, capacity=1)  # Resource representing the human (can handle 1 task at a time)
        self.mtbf = mtbf                              # Mean Time Between Failures (average uptime before breakdown)
        self.mttr = mttr_range                        # Range of repair times when breakdown happens
        self.down = False                             # Flag to track whether the assembly cell is down
        if self.mtbf:                                 # If MTBF is provided, start breakdown generator
            env.process(self._breakdowns())

    def _breakdowns(self):
        while True:                                   # Continuous loop to simulate random breakdowns
            ttf = random.expovariate(1 / self.mtbf)   # Sample a random time-to-failure from exponential distribution
            yield self.env.timeout(ttf)               # Wait until that failure occurs
            down_start = self.env.now                 # Record failure start time
            self.down = True                          # Mark station as down
            rdur = u(*self.mttr)                      # Draw a random repair duration from uniform range
            yield self.env.timeout(rdur)              # Wait for repair to finish
            self.down = False                         # Mark station as operational again
            stats.add_down("Assembly", down_start, self.env.now)  # Log downtime interval into stats

    def _use(self, eid, who, dur, step):
        if who == "robot":                            # If only the robot is involved in this step
            with self.robot.request() as r:           # Request the robot resource
                yield r                               # Wait until robot is available
                start = self.env.now                  # Record start time
                opcua_write(f"Assembly.Robot.{step}", "start")  # Log OPC UA tag (robot step started)
                yield self.env.timeout(dur)           # Wait for the step duration
                opcua_write(f"Assembly.Robot.{step}", "done")   # Log OPC UA tag (robot step finished)
                stats.add_busy("Robot", start, self.env.now)    # Record robot busy interval

        elif who == "human":                          # If only the human is involved
            with self.human.request() as h:           # Request the human resource
                yield h                               # Wait until human is available
                start = self.env.now                  # Record start time
                opcua_write(f"Assembly.Human.{step}", "start")  # Log OPC UA tag (human step started)
                yield self.env.timeout(dur)           # Wait for the step duration
                opcua_write(f"Assembly.Human.{step}", "done")   # Log OPC UA tag (human step finished)
                stats.add_busy("Human", start, self.env.now)    # Record human busy interval

        elif who == "both":                           # If both robot & human are required simultaneously
            with self.robot.request() as r, self.human.request() as h:  # Request both resources
                yield r & h                           # Wait until both are available
                start = self.env.now                  # Record start time
                opcua_write(f"Assembly.Step.{step}", "start")  # Log OPC UA tag (joint step started)
                yield self.env.timeout(dur)           # Wait for the step duration
                opcua_write(f"Assembly.Step.{step}", "done")   # Log OPC UA tag (joint step finished)
                stats.add_busy("Robot", start, self.env.now)   # Record robot busy interval
                stats.add_busy("Human", start, self.env.now)   # Record human busy interval

    def process(self, eid):                           # Defines the sequence of micro-steps in Assembly
        yield from self._use(eid,"both",_parallel(u(*ROBOT_PNP_ANG),u(*HUMAN_PLACE_BASE)),"A")   # Step A: place base + angular
        yield from self._use(eid,"both",_parallel(u(*ROBOT_PNP_ANG),u(*HUMAN_INS_ANG_AND_BOTTOM)),"B")  # Step B: insert bottom + angular
        yield from self._use(eid,"both",_parallel(u(*ROBOT_PREP_HOLD),u(*HUMAN_INS_ANG_AND_SIDE1)),"C") # Step C: prepare hold + insert side
        yield from self._use(eid,"both",u(*HUMAN_SCREW_SIDE),"D")  # Step D: human screws while robot holds
        yield from self._use(eid,"both",_parallel(u(*ROBOT_PNP_ANG)+u(*ROBOT_PNP_ANG),u(*HUMAN_MARKER_ROTATE)),"E") # Step E: place angulars + rotate
        yield from self._use(eid,"both",_parallel(u(*ROBOT_PREP_HOLD),u(*HUMAN_INS_ANG34_AND_SIDE2)),"F") # Step F: prepare hold + insert side2
        yield from self._use(eid,"both",u(*HUMAN_SCREW_SIDE),"G")  # Step G: human screws while robot holds
        yield from self._use(eid,"both",_parallel(u(*ROBOT_RETURN_HOME),u(*HUMAN_PLACE_ON_CONV)),"H") # Step H: robot return + place box on conveyor


#### Buffers 
def qc_pack_time():
    insp = 60 / u(*QC_INSPECTION_RATE)       # Visual insp: 3–4 units/min -> 15–20 s per unit (approx.)
    pack = u(PACK_TIME_MIN, PACK_TIME_MAX) # Target guide says ≈ 4 minutes total; we keep stochastic but not forced to 240 s
    return insp + pack

def putaway_time():    ###Storage  ###To put a box into storage
    return u(*PUTAWAY_TIME)


# Line setup ## main flow of a box through the system
def build_line(env):
    cnc = Station(env, "CNC", cnc_time, mtbf=CNC_MTBF, mttr_range=CNC_MTTR)
    asm = AssemblyCell(env, mtbf=ASM_MTBF, mttr_range=ASM_MTTR)  # <-- use AssemblyCell
    qc  = Station(env, "QC_Pack", qc_pack_time)  # QC remains a simple Station
    stg = Station(env, "Storage", putaway_time)
    return cnc, asm, qc, stg

# Buffers 
def init_buffers(env):
    # Initialize three intermediate buffers (queues) with limited capacity
    return (simpy.Store(env,capacity=10),   # Buffer between CNC and Assembly (max 10 panels)
            simpy.Store(env,capacity=5),    # Buffer between Assembly and QC (max 5 assemblies)
            simpy.Store(env,capacity=20))   # Buffer between QC and Storage (max 20 finished units)

# ---- Flow ----
def cnc_flow(env,cnc,buf):
    eid=0                                     # Entity ID counter (unique ID for each part/panel)
    while True:                               # Infinite loop to keep producing
        eid+=1                                # Increment entity ID for each new part
        stats.mark_enter(env,eid)             # Log entry of new part into the system
        yield from cnc.process(eid)           # Process entity at CNC (cutting etc.)
        yield buf.put(eid)                    # Place completed part into buffer (to Assembly)
        yield env.timeout(90)                 # Wait ~90s before creating next entity (interarrival time)

def asm_flow(env,asm,buf_in,buf_out):
    while True:                               # Infinite loop to process parts in Assembly
        eid=yield buf_in.get()                # Take next part from CNC→Assembly buffer
        yield from asm.process(eid)           # Perform Assembly process (robot + human steps)
        if random.random()<OP_ERROR_P_ASM:    # With small probability, assembly error occurs
            stats.reworked+=1                 # Log rework event
            yield env.timeout(10)             # Admin/handling delay before reassembly
            yield from asm.process(eid)       # Re-run assembly on the same entity
        yield buf_out.put(eid)                # Send assembled entity into Assembly→QC buffer

def qc_flow(env,qc,buf_in,buf_out):
    while True:                               # Infinite loop for QC inspection & packaging
        eid=yield buf_in.get()                # Take next part from Assembly→QC buffer
        yield from qc.process(eid)            # Perform QC & packaging
        if random.random()<OP_ERROR_P_QC:     # With small probability, QC rejects the part
            stats.qc_rejects+=1               # Log QC reject event
            yield env.timeout(20)             # Handling/transport delay for rework
            yield from qc.process(eid)        # Repeat QC after rework
        yield buf_out.put(eid)                # Send inspected entity into QC→Storage buffer

def stg_flow(env,stg,buf_in):
    while True:                               # Infinite loop for Storage stage
        eid=yield buf_in.get()                # Take next part from QC→Storage buffer
        yield from stg.process(eid)           # Put the part into storage
        stats.mark_complete(env,eid)          # Mark entity as completed in the system

def wip_sampler(env):
    while True:                               # Infinite loop for sampling WIP
        stats.sample_wip(env)                 # Record current Work-In-Progress
        yield env.timeout(300)                # Sample every 300s (5 minutes)

################### Run

random.seed(RANDOM_SEED)                      # Fix random seed for reproducibility
env=simpy.Environment()                       # Create simulation environment
cnc,asm,qc,stg=build_line(env)                # Build line with stations: CNC, Assembly, QC, Storage
buf_cnc_to_asm,buf_asm_to_qc,buf_qc_to_stg=init_buffers(env)  # Initialize finite buffers

# Launch processes for each stage of the flow
env.process(cnc_flow(env,cnc,buf_cnc_to_asm))         # CNC → buffer
env.process(asm_flow(env,asm,buf_cnc_to_asm,buf_asm_to_qc))   # Assembly → buffer
env.process(qc_flow(env,qc,buf_asm_to_qc,buf_qc_to_stg))      # QC → buffer
env.process(stg_flow(env,stg,buf_qc_to_stg))                  # Storage
env.process(wip_sampler(env))                                 # WIP sampler
env.run(until=SHIFT_SECONDS)                                  # Run full 8h shift

###################### Report

print("\n=== SHIFT RESULTS (8h) ===")               # Section header
print(f"Completed boxes: {stats.completed} (target ~{TARGET_PER_SHIFT})")  # Throughput vs target
if stats.cycle_times:                               # If there are completed parts, compute stats
    mean_ct=statistics.mean(stats.cycle_times)      # Average cycle time per unit
    p90_ct=statistics.quantiles(stats.cycle_times,n=10)[8]  # 90th percentile cycle time
    print(f"Cycle time (avg): {mean_ct:,.1f}s | P90: {p90_ct:,.1f}s")  # Print cycle time metrics
print(f"Reworks: {stats.reworked}, QC rejects: {stats.qc_rejects}")    # Print error/rework counts
for name in ["CNC","Robot","Human","QC_Pack","Storage"]:               # Utilization report per resource
    busy=stats.util_busy[name]; down=stats.util_down[name]
    print(f"- {name:10s} | Busy {busy:,.0f}s Down {down:,.0f}s Util≈{busy/SHIFT_SECONDS:0.2%}")
if stats.wip_samples:                                # If WIP samples exist, compute avg WIP
    avg_wip=statistics.mean([w for _,w in stats.wip_samples])
    print(f"Avg WIP: {avg_wip:0.2f}")                # Print average WIP
UPH=stats.completed/(SHIFT_SECONDS/3600)             # Units per hour (throughput)
print(f"Throughput ≈ {UPH:0.1f} units/hour")         # Print throughput



