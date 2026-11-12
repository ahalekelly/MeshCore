# /// script
# requires-python = ">=3.12"
# dependencies = ["meshcore"]
# ///
"""Short off-channel carrier bursts against a victim radio, counting receive lockups.

Usage: uv run bursttest.py VICTIM_PORT TX_PORT OUT_PREFIX --bursts N [--block 20] [--bits ab|0|1|keep]
       [--arm none,20000:0,1000000:2000] [--khz 907525[,908025,...]] [--dbm -9[,-8,...]] [--burst-ms 300]

Each burst: TX_PORT sends an unmodulated carrier (`radiotx`); VICTIM's live RSSI is sampled during
it and for 1.5 s after. Locked = the last samples all stay at or above -95 dBm. A locked victim is
watched for HOLD_S while both radios count packets, then cleared with `radiofix rx` (then
`radiofix agc`). --bits keep never writes 0x08B5 and logs the bit as read at each block start;
--bits ab alternates bit 0 of register 0x08B5 per block of bursts, random order
within each pair of blocks. --arm cycles per burst: "none" (VICTIM left alone), or PERIOD_US:POST_US,
which runs `radioarm PERIOD_US 3000 POST_US 600` on VICTIM: it restarts receive every PERIOD_US until the
carrier appears, logs the onset time after the last restart (`phase`), and with POST_US > 0 restarts once
more that long after onset. VICTIM is busy while armed, so those bursts have no host RSSI samples during it.
--khz and --dbm with several values send every (frequency, power) pair in shuffled rounds, one burst each per round.
Writes OUT_PREFIX-bursts.csv (one row per burst) and OUT_PREFIX-traces.csv (every RSSI sample).
"""
import argparse
import asyncio
import csv
import random
import time
from statistics import median

from meshcore import EventType, MeshCore

ALLOWED_KHZ = {907525, 908025, 914025, 915525}
OFFSET_SWEEP_KHZ = {903000, 905900, 908525, 912300, 921000, 925000}  # approved only for the 11-11 offset sweep
LOCK_DBM = -95
HOLD_S = 5


def args_():
    p = argparse.ArgumentParser()
    p.add_argument("victim")
    p.add_argument("tx")
    p.add_argument("out")
    p.add_argument("--bursts", type=int, required=True)
    p.add_argument("--block", type=int, default=20)
    p.add_argument("--bits", default="ab", choices=["ab", "0", "1", "keep"])
    p.add_argument("--arm", default="none")
    p.add_argument("--khz", default="907525")
    p.add_argument("--dbm", default="-9")
    p.add_argument("--burst-ms", type=int, default=300)
    a = p.parse_args()
    a.khz = [int(k) for k in a.khz.split(",")]
    if not set(a.khz) <= ALLOWED_KHZ | OFFSET_SWEEP_KHZ:
        p.error(f"--khz must be among {sorted(ALLOWED_KHZ | OFFSET_SWEEP_KHZ)}")
    a.dbm = [int(d) for d in a.dbm.split(",")]
    if "RAK3401" in a.tx and a.dbm != [-9]:
        p.error("the 1 W board transmits only at -9")
    if max(a.dbm) > 0 or a.burst_ms > 5000:
        p.error("at most 0 dBm and 5000 ms")
    a.arm = [None if x == "none" else tuple(map(int, x.split(":"))) for x in a.arm.split(",")]
    return a


async def main():
    a = args_()
    victim = await MeshCore.create_serial(a.victim, 115200, default_timeout=30)
    tx = await MeshCore.create_serial(a.tx, 115200, default_timeout=30)
    heard = {"victim": [], "ref": []}
    victim.subscribe(EventType.RX_LOG_DATA, lambda e: heard["victim"].append(time.monotonic()))
    tx.subscribe(EventType.RX_LOG_DATA, lambda e: heard["ref"].append(time.monotonic()))

    async def cli(radio, command):
        event = await radio.commands.run_cli_command(command)
        if event.type == EventType.ERROR:
            raise RuntimeError(f"{command}: {event.payload}")
        return event.payload["text"]

    trace = []  # (t_monotonic, median dBm or None, raw rs)

    async def sample():
        fields = dict(p.split("=", 1) for p in (await cli(victim, "radiodiag2")).split())
        data = bytes.fromhex(fields["rs"])
        value = None if 255 in data else median(-b / 2 for b in data)
        trace.append((time.monotonic(), value, fields["rs"]))
        return value, fields["stats"]

    async def watch(seconds):
        values = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            values.append((await sample())[0])
        return values

    def locked(values):
        late = [v for v in values[-8:] if v is not None]
        return len(late) >= 5 and min(late) >= LOCK_DBM

    async def reg(addr, n):
        return (await cli(victim, f"radioreg {addr} {n}")).split(":")[1]

    def near(times, t):
        before = [x for x in times if x <= t]
        after = [x for x in times if x > t]
        return (round((t - before[-1]) * 1000) if before else "", round((after[0] - t) * 1000) if after else "")

    bf = open(f"{a.out}-bursts.csv", "a", newline="")
    tf = open(f"{a.out}-traces.csv", "a", newline="")
    bw, tw = csv.writer(bf), csv.writer(tf)
    if bf.tell() == 0:
        bw.writerow(["time", "burst", "block", "bit", "arm", "khz", "dbm", "burst_ms", "pre_dbm", "stats_pre", "stats_post",
                     "victim_since_ms", "victim_next_ms", "ref_since_ms", "ref_next_ms", "victim_during", "ref_during",
                     "tx_reply_ms", "arm_reply", "onset_ms", "during_min", "during_max", "after", "locked", "reg_8bf", "hold_victim", "hold_ref", "cure"])
    if tf.tell() == 0:
        tw.writerow(["burst", "t_ms", "dbm", "rs"])
    bf.flush()
    first_burst = sum(1 for _ in open(f"{a.out}-bursts.csv")) - 1  # appended runs continue the numbering
    original = await reg("08B5", 1)
    plan, bit, counts, rounds = [], None, {}, []
    for i in range(first_burst, first_burst + a.bursts):
        if (i - first_burst) % a.block == 0:
            if a.bits == "keep":
                bit = int(await reg("08B5", 1), 16) & 1
            elif a.bits == "ab":
                if not plan:
                    first = random.getrandbits(1)
                    plan = [first, first ^ 1]
                bit = plan.pop(0)
            else:
                bit = int(a.bits)
            if a.bits != "keep":
                before = int(await reg("08B5", 1), 16)
                await cli(victim, f"radiowreg 08B5 {before & 0xFE | bit:02X}")
                await cli(victim, "radiofix rx")
                if int(await reg("08B5", 1), 16) & 1 != bit:
                    raise RuntimeError("bit did not stick")
        await asyncio.sleep(random.uniform(0.2, 0.6))
        arm = a.arm[(i - first_burst) % len(a.arm)]
        trace.clear()
        pre, stats_pre = await sample()
        armed, during = None, []
        if arm is not None:
            armed = asyncio.create_task(cli(victim, f"radioarm {arm[0]} 3000 {arm[1]} 600"))
            await asyncio.sleep(random.uniform(0.05, 0.2))
        start = time.monotonic()
        if not rounds:
            pairs = [(k, d) for k in a.khz for d in a.dbm]
            rounds = random.sample(pairs, len(pairs))
        khz, dbm = rounds.pop()
        carrier = asyncio.create_task(cli(tx, f"radiotx {khz} {dbm} {a.burst_ms}"))
        if armed is not None:
            armed = await armed
            if "timeout" in armed:
                raise RuntimeError(f"radioarm saw no carrier: {armed}")
        while not carrier.done():
            during.append((await sample())[0])
        if (reply := await carrier) != "result=0":
            raise RuntimeError(f"carrier failed: {reply}")
        stop = time.monotonic()
        after = await watch(1.5)
        stats_post = (await sample())[1]
        is_locked = locked(after)
        dv = [v for v in during if v is not None]
        row = [time.strftime("%Y-%m-%dT%H:%M:%S"), i, i // a.block, bit, "" if arm is None else f"{arm[0]}:{arm[1]}", khz, dbm, a.burst_ms,
               pre, stats_pre, stats_post, *near(heard["victim"], start), *near(heard["ref"], start),
               sum(start <= t <= stop for t in heard["victim"]), sum(start <= t <= stop for t in heard["ref"]),
               round((stop - start) * 1000), armed or "",
               next((round((t - start) * 1000, 1) for t, v, _ in trace if t > start and v is not None and v >= -90), ""),
               min(dv, default=""), max(dv, default=""), " ".join("FF" if v is None else f"{v:.0f}" for v in after[:20]), int(is_locked)]
        counts.setdefault((bit, arm, khz, dbm), [0, 0])[0] += 1
        if is_locked:
            counts[(bit, arm, khz, dbm)][1] += 1
            regs = await reg("08BF", 2)
            mark = time.monotonic()
            hold = await watch(HOLD_S)
            row += [regs, sum(t >= mark for t in heard["victim"]), sum(t >= mark for t in heard["ref"])]
            cure = "self"
            if locked(hold):
                await cli(victim, "radiofix rx")
                cure = "rx"
                if locked(await watch(1.5)):
                    await cli(victim, "radiofix agc")
                    cure = "agc"
                    if locked(await watch(3)):
                        bw.writerow(row + ["FAILED"])
                        tw.writerows([i, round((t - start) * 1000), "" if v is None else v, rs] for t, v, rs in trace)
                        bf.flush()
                        tf.flush()
                        raise RuntimeError("victim still locked after radiofix agc; left as is")
            row.append(cure)
            print(f"{row[0]} burst {i} bit {bit} arm {arm} {armed or ''} LOCKED victim_since {row[11]} ref_since {row[13]} ref_next {row[14]} "
                  f"onset {row[19]} during {row[20]}..{row[21]} regs {regs} hold v{row[25]} r{row[26]} cure {cure}", flush=True)
        else:
            row += ["", "", "", ""]
        bw.writerow(row)
        tw.writerows([i, round((t - start) * 1000), "" if v is None else v, rs] for t, v, rs in trace)
        bf.flush()
        tf.flush()
        if (i - first_burst) % 20 == 19:
            print(time.strftime("%T"), f"{i + 1} bursts", {str(k): v for k, v in counts.items()}, flush=True)
    if a.bits != "keep":
        await cli(victim, f"radiowreg 08B5 {original}")
        await cli(victim, "radiofix rx")
    print("final 08B5", await reg("08B5", 1), "restored to the starting value", original)
    for (bit, arm, khz, dbm), (n, k) in sorted(counts.items(), key=str):
        print(f"bit {bit} arm {arm} {khz} kHz {dbm} dBm: {k} locked of {n}")
    await victim.disconnect()
    await tx.disconnect()

asyncio.run(main())
