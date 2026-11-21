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


def save(baseline, path):
    with open(path, 'w') as f:
        json.dump({'version': 1, 'streams': baseline}, f, indent=2, sort_keys=True)


def load(path):
    with open(path) as f:
        data = json.load(f)
    return data['streams']
