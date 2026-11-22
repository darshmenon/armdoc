"""Per-robot baselines: learn what normal looks like on this robot, then flag deviations. Pure statistics, no ROS, no training data.

`learn` turns message arrival times into a rate (median messages per second) and a jitter (median absolute deviation of that count). `check` compares the recent rate with that baseline."""
import json
import statistics

MIN_SECONDS = 5         # whole seconds of data needed to trust a stream's baseline
DROP_RATIO = 0.5        # recent rate below this fraction of the baseline is a drop
SPIKE_RATIO = 2.0       # recent rate above this multiple is a spike
WINDOW_S = 6.0          # look-back used by `check`


def per_second_counts(times):
    """Messages per whole second, full seconds only (the first and last are partial)."""
    times = sorted(times)
    if len(times) < 2:
        return []
    t0 = times[0]
    counts = {}
    for t in times:
        counts[int(t - t0)] = counts.get(int(t - t0), 0) + 1
    last = int(times[-1] - t0)
    return [counts.get(i, 0) for i in range(1, last)]


def learn(arrivals):
    """{stream: [times]} -> {stream: {'hz', 'jitter', 'n'}}. The rate is the median messages per second, so bursts of
    messages in one instant (TF) and a few stalls do not skew it. Streams seen for too few seconds are left out."""
    out = {}
    for stream, times in arrivals.items():
        c = per_second_counts(list(times))
        if len(c) < MIN_SECONDS:
            continue
        med = statistics.median(c)
        if med <= 0:
            continue
        mad = statistics.median(abs(x - med) for x in c)
        out[stream] = {'hz': round(float(med), 3), 'jitter': round(float(mad), 3), 'n': len(times)}
    return out


def recent_rate(times, now, window=WINDOW_S):
    t = [x for x in times if now - x <= window]
    if len(t) < 3:
        return None
    return (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else None


def check(baseline, arrivals, now, window=WINDOW_S):
    """Findings as (level, code, msg). A stream with no recent messages is not reported here: the stale checks own that."""
    found = []
    for stream, b in sorted(baseline.items()):
        r = recent_rate(arrivals.get(stream, []), now, window)
        if r is None:
            continue
        if r < b['hz'] * DROP_RATIO:
            found.append(('WARN', 'RATE_DROP', f'{stream} runs at {r:.1f} Hz, this robot normally gives {b["hz"]:.1f} Hz '
                          f'({r / b["hz"]:.0%} of normal)'))
        elif r > b['hz'] * SPIKE_RATIO:
            found.append(('WARN', 'RATE_SPIKE', f'{stream} runs at {r:.1f} Hz, this robot normally gives {b["hz"]:.1f} Hz '
                          f'({r / b["hz"]:.1f}x normal)'))
    return found


def parse_expect(items):
    """['scan:10', 'odom=20'] -> {'scan': {'hz': 10.0, 'jitter': 0.0, 'n': 0}}: rates stated with --expect, used like a
    learned baseline and overriding it for the same stream. Raises ValueError on a bad item."""
    out = {}
    for item in items or ():
        name, sep, hz = item.replace('=', ':').rpartition(':')
        if not sep or not name.strip().strip('/'):
            raise ValueError(f'--expect wants STREAM:HZ (e.g. joint_states:50), got {item!r}')
        try:
            rate = float(hz)
        except ValueError:
            raise ValueError(f'--expect {item!r}: {hz!r} is not a number') from None
        if rate <= 0:
            raise ValueError(f'--expect {item!r}: rate must be > 0')
        out[name.strip().strip('/')] = {'hz': rate, 'jitter': 0.0, 'n': 0}
    return out


def save(baseline, path):
    with open(path, 'w') as f:
        json.dump({'version': 1, 'streams': baseline}, f, indent=2, sort_keys=True)


def load(path):
    """Read a baseline file written by `save`. Raises ValueError with a readable message (never a bare traceback, and never
    a malformed entry that only crashes later inside `check`) for a missing file, bad JSON, no `streams`, or an entry
    without a positive numeric `hz`."""
    try:
        with open(path) as f:
            data = json.load(f)
    except OSError as e:
        raise ValueError(f'cannot read baseline file {path!r}: {e.strerror or e}') from None
    except ValueError as e:
        raise ValueError(f'baseline file {path!r} is not valid JSON ({e}); re-create it with --learn') from None
    streams = data.get('streams') if isinstance(data, dict) else None
    if not isinstance(streams, dict):
        raise ValueError(f'baseline file {path!r} has no "streams" object; re-create it with --learn')
    for name, entry in streams.items():
        hz = entry.get('hz') if isinstance(entry, dict) else None
        if isinstance(hz, bool) or not isinstance(hz, (int, float)) or not hz > 0:
            raise ValueError(f'baseline file {path!r}: stream {name!r} needs a positive numeric "hz", got {entry!r}')
    return streams
