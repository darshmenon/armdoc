"""Turn an `armdoc --record` JSONL file into a joint-position timeline with finding bands.

  armdoc report run.jsonl --html report.html [--png report.png]

The HTML is one self-contained file (inline SVG + a little JS, no network). ROS is not needed.
"""
import argparse
import bisect
import html
import json
import math
import sys

# categorical slots 1-6 (blue, orange, aqua, yellow, magenta, green), light / dark steps
SERIES_LIGHT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
SERIES_DARK = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"]
ERR_COLOR, WARN_COLOR = "#d03b3b", "#fab219"
GAP = 1.5          # s without joint data that breaks a line


# ---------------------------------------------------------------- loading and analysis
def load(path):
    meta, limits, joints, samples = {}, {}, [], []
    with open(path) as fh:
        for line in fh:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            k = r.get("type")
            if k == "meta":
                meta = r
                limits.update(r.get("limits") or {})
            elif k == "limits":
                limits.update(r.get("limits") or {})
            elif k == "joints":
                joints.append((r["t"], r["pos"]))
            elif k == "findings":
                samples.append((r["t"], r.get("findings", []), r.get("ctrl", {})))
    names = []
    for _t, p in joints:
        for n in p:
            if n not in names:
                names.append(n)
    t_end = max([t for t, _ in joints] + [t for t, _, _ in samples] + [0.0])
    return {"meta": meta, "limits": limits, "joints": joints, "names": names, "samples": samples, "t_end": t_end}


def spans(samples, t_end):
    """Intervals during which each finding code was continuously reported.
    A sample at time t describes [t, next sample); a span ends at the first sample without the code."""
    open_, out = {}, []
    for i, (t, fs, _c) in enumerate(samples):
        now = {f["code"]: f for f in fs}
        for code, f in now.items():
            if code not in open_:
                open_[code] = {"code": code, "level": f["level"], "start": t, "end": t_end, "msg": f["msg"]}
            elif f["level"] == "ERROR":
                open_[code]["level"] = "ERROR"
        for code in list(open_):
            if code not in now:
                open_[code]["end"] = t
                out.append(open_.pop(code))
    out.extend(open_.values())
    return sorted(out, key=lambda s: (s["start"], s["code"]))


def ctrl_spans(samples, t_end):
    """Intervals where a controller that had been active was reported in another state. Controllers that
    were never active (an unused effort controller, say) are not a fault and are left out."""
    open_, out, seen = {}, [], set()
    for t, _f, ctrl in samples:
        for n, st in ctrl.items():
            if st == "active":
                seen.add(n)
            if st != "active" and n in seen and n not in open_:
                open_[n] = {"name": n, "state": st, "start": t, "end": t_end}
            elif st == "active" and n in open_:
                open_[n]["end"] = t
                out.append(open_.pop(n))
    out.extend(open_.values())
    return sorted(out, key=lambda s: s["start"])


def series(data):
    """{joint: [(t, value or None)]} with None inserted at gaps so lines break where data is missing."""
    out = {n: [] for n in data["names"]}
    last = {n: None for n in data["names"]}
    for t, p in data["joints"]:
        for n in data["names"]:
            if n not in p:
                continue
            if last[n] is not None and t - last[n] > GAP:
                out[n].append((last[n] + 0.01, None))
            out[n].append((t, p[n]))
            last[n] = t
    return out


def nice_ticks(lo, hi, n=5):
    if hi - lo < 1e-9:
        lo, hi = lo - 0.5, hi + 0.5
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    a = math.floor(lo / step) * step
    ticks, v = [], a
    while True:
        ticks.append(round(v, 10))
        if v >= hi - 1e-9:          # last tick must reach the top of the data
            return ticks
        v += step


def summary(data):
    sp = spans(data["samples"], data["t_end"])
    return {"errors": sorted({s["code"] for s in sp if s["level"] == "ERROR"}),
            "warnings": sorted({s["code"] for s in sp if s["level"] == "WARN"}), "spans": sp}


def label_rows(sp, x_of, char_w=6.4, pad=16):
    """Greedy rows so band labels above the plot never overlap. Returns [row per span], row count."""
    ends, rows = [], []
    for s in sp:
        x = x_of(s["start"])
        w = len(s["code"]) * char_w + 16
        for r, e in enumerate(ends):
            if x >= e + pad:
                ends[r] = x + w
                rows.append(r)
                break
        else:
            ends.append(x + w)
            rows.append(len(ends) - 1)
    return rows, max(1, len(ends))


def fmt_t(t):
    return f"{t:.1f}s"


# ---------------------------------------------------------------- HTML
CSS = """
:root{color-scheme:light;--bg:#fcfcfb;--ink:#0b0b0b;--ink2:#52514e;--muted:#8a8984;--grid:#e4e3df;--line:#c9c8c3;--card:#f4f3f0;
--err:#d03b3b;--warn:#fab219;%LIGHT%}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--bg:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;
--muted:#8f8e86;--grid:#2d2d2b;--line:#4a4a47;--card:#232322;%DARK%}}
:root[data-theme="dark"]{color-scheme:dark;--bg:#1a1a19;--ink:#f0efec;--ink2:#c3c2b7;--muted:#8f8e86;--grid:#2d2d2b;--line:#4a4a47;--card:#232322;%DARK%}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}
main{max-width:1040px;margin:0 auto;padding:24px 16px 48px}
h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:28px 0 8px}
.sub{color:var(--ink2);font-size:13px}
.chips{display:flex;flex-wrap:wrap;gap:8px;margin:14px 0}
.chip{border:1px solid var(--line);border-radius:999px;padding:3px 12px;font-size:13px;background:var(--card)}
.chip b{font-weight:600}.ok{color:var(--ink2)}
.card{background:var(--card);border-radius:12px;padding:12px 8px 4px;position:relative}
svg{width:100%;height:auto;display:block;overflow:visible}
svg text{font-family:inherit;fill:var(--ink2);font-size:11px}
.grid{stroke:var(--grid);stroke-width:1}.axis{stroke:var(--line);stroke-width:1}
.series path{fill:none;stroke-width:2;stroke-linejoin:round}.series.off{display:none}
.lim{stroke-width:1;stroke-dasharray:4 4;opacity:.5}
.band-e{fill:var(--err);fill-opacity:.16}.band-w{fill:var(--warn);fill-opacity:.24}
.bl{fill:var(--ink)!important;font-weight:600}
.lane-r{fill:var(--muted);fill-opacity:.25}
.legend{display:flex;flex-wrap:wrap;gap:6px 14px;padding:8px 12px 10px;font-size:13px}
.legend button{all:unset;cursor:pointer;display:flex;align-items:center;gap:6px;color:var(--ink);padding:2px 4px;border-radius:6px}
.legend button:focus-visible{outline:2px solid var(--ink2)}
.legend i{width:14px;height:3px;border-radius:2px;display:inline-block}.legend button.off{opacity:.4;text-decoration:line-through}
#xh{stroke:var(--ink2);stroke-width:1;display:none}
#tip{position:absolute;pointer-events:none;display:none;background:var(--bg);color:var(--ink);border:1px solid var(--line);
border-radius:8px;padding:6px 10px;font-size:12px;line-height:1.5;box-shadow:0 2px 8px #0003;z-index:2;white-space:nowrap}
#tip .e{color:var(--err)}#tip .w{color:var(--ink)}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--grid);vertical-align:top}
th{color:var(--ink2);font-weight:600}td.n{font-variant-numeric:tabular-nums;white-space:nowrap}
.lv{white-space:nowrap}details{margin-top:12px}summary{cursor:pointer;color:var(--ink2)}
.scroll{overflow-x:auto}
"""


def build_html(data, title=None):
    W, H_PLOT = 1000, 300
    L, R, B = 58, 16, 56
    sp = spans(data["samples"], data["t_end"])
    cs = ctrl_spans(data["samples"], data["t_end"])
    ser = series(data)
    names = data["names"]
    t_end = max(data["t_end"], 1.0)
    allv = [v for pts in ser.values() for _t, v in pts if v is not None]
    lo, hi = (min(allv), max(allv)) if allv else (-1.0, 1.0)
    pad = (hi - lo) * 0.06 or 0.5
    yt = nice_ticks(lo - pad, hi + pad, 8)
    y0, y1 = yt[0], yt[-1]
    x_of = lambda t: L + (W - L - R) * t / t_end
    rows, nrows = label_rows(sp, x_of)
    top = 14 + 16 * nrows
    lane_h = 22 if cs else 0
    H = top + H_PLOT + B + lane_h
    y_of = lambda v: top + H_PLOT * (1 - (v - y0) / (y1 - y0))
    xt = nice_ticks(0, t_end, 8)
    g = []
    for v in yt:
        g.append(f'<line class="grid" x1="{L}" x2="{W - R}" y1="{y_of(v):.1f}" y2="{y_of(v):.1f}"/>'
                 f'<text x="{L - 8}" y="{y_of(v) + 4:.1f}" text-anchor="end">{v:g}</text>')
    for t in xt:
        if 0 <= t <= t_end:
            g.append(f'<text x="{x_of(t):.1f}" y="{top + H_PLOT + 18}" text-anchor="middle">{t:g}</text>')
    g.append(f'<text x="{(L + W - R) / 2:.0f}" y="{top + H_PLOT + 36}" text-anchor="middle">time since start (s)</text>')
    g.append(f'<text transform="translate(14 {top + H_PLOT / 2:.0f}) rotate(-90)" text-anchor="middle">joint position (rad)</text>')
    g.append(f'<line class="axis" x1="{L}" x2="{W - R}" y1="{top + H_PLOT}" y2="{top + H_PLOT}"/>')
    # finding bands + labels above the plot
    for s, r in zip(sp, rows):
        x_a, x_b = x_of(s["start"]), max(x_of(s["end"]), x_of(s["start"]) + 2)
        cls = "band-e" if s["level"] == "ERROR" else "band-w"
        mark = "●" if s["level"] == "ERROR" else "▲"
        tip = html.escape(f'{s["level"]} {s["code"]} {fmt_t(s["start"])}–{fmt_t(s["end"])}: {s["msg"]}')
        g.append(f'<g><title>{tip}</title><rect class="{cls}" x="{x_a:.1f}" y="{top}" width="{x_b - x_a:.1f}" height="{H_PLOT}"/>'
                 f'<line x1="{x_a:.1f}" x2="{x_a:.1f}" y1="{14 + 16 * r + 3}" y2="{top}" stroke="{ERR_COLOR if s["level"] == "ERROR" else WARN_COLOR}" stroke-width="1.5"/>'
                 f'<text class="bl" x="{x_a + 3:.1f}" y="{14 + 16 * r}">{mark} {html.escape(s["code"])}</text></g>')
    # limits (dashed) are not drawn: ten of them would bury the data
    for i, n in enumerate(names):
        pts, d = ser[n], ""
        pen = False
        for t, v in pts:
            if v is None:
                pen = False
                continue
            d += f'{"L" if pen else "M"}{x_of(t):.1f} {y_of(v):.1f}'
            pen = True
        g.append(f'<g class="series" data-j="{i}"><path d="{d}" style="stroke:var(--s{i % 8})"/></g>')
    # controller lane
    if cs:
        ly = top + H_PLOT + 50
        g.append(f'<text x="{L - 8}" y="{ly + 11}" text-anchor="end">controllers</text>')
        for c in cs:
            xa, xb = x_of(c["start"]), max(x_of(c["end"]), x_of(c["start"]) + 2)
            lab = html.escape(f'{c["name"]}: {c["state"]}')
            g.append(f'<g><title>{lab} {fmt_t(c["start"])}–{fmt_t(c["end"])}</title>'
                     f'<rect class="lane-r" x="{xa:.1f}" y="{ly}" width="{xb - xa:.1f}" height="14" rx="3"/>'
                     f'<text x="{xa + 4:.1f}" y="{ly + 11}">{lab}</text></g>')
    g.append(f'<line id="xh" x1="0" x2="0" y1="{top}" y2="{top + H_PLOT}"/>')
    svg = (f'<svg id="chart" viewBox="0 0 {W} {H}" role="img" aria-label="joint position timeline with finding bands">'
           + "".join(g) + "</svg>")

    legend = "".join(f'<button data-j="{i}" aria-pressed="true"><i style="background:var(--s{i % 8})"></i>{html.escape(n)}</button>'
                     for i, n in enumerate(names))
    sm = summary(data)
    chips = []
    if not sp:
        chips.append('<span class="chip ok">✓ <b>no findings</b>, arm looked healthy</span>')
    for c in sm["errors"]:
        chips.append(f'<span class="chip">● <b>ERROR</b> {html.escape(c)}</span>')
    for c in sm["warnings"]:
        if c not in sm["errors"]:
            chips.append(f'<span class="chip">▲ <b>WARN</b> {html.escape(c)}</span>')
    trs = "".join(
        f'<tr><td class="lv">{"● ERROR" if s["level"] == "ERROR" else "▲ WARN"}</td><td>{html.escape(s["code"])}</td>'
        f'<td class="n">{s["start"]:.1f}</td><td class="n">{s["end"]:.1f}</td><td class="n">{s["end"] - s["start"]:.1f}</td>'
        f'<td>{html.escape(s["msg"])}</td></tr>' for s in sp) or '<tr><td colspan="6">none</td></tr>'
    # 1 Hz joint table
    tab_rows, nxt = [], 0.0
    for t, p in data["joints"]:
        if t >= nxt:
            tab_rows.append("<tr><td class=\"n\">%.1f</td>%s</tr>" % (t, "".join(
                f'<td class="n">{p[n]:.3f}</td>' if n in p else "<td>–</td>" for n in names)))
            nxt = t + 1.0
    joint_tbl = ("<tr><th>t (s)</th>" + "".join(f"<th>{html.escape(n)}</th>" for n in names) + "</tr>"
                 + "".join(tab_rows[:600]))
    # data for the hover layer
    hv = {"L": L, "R": R, "W": W, "t_end": t_end, "names": names,
          "t": [t for t, _ in data["joints"]], "p": [[p.get(n) for n in names] for _t, p in data["joints"]],
          "spans": [{"c": s["code"], "l": s["level"], "a": s["start"], "b": s["end"]} for s in sp]}
    meta = data["meta"]
    sub = f'recorded {html.escape(meta.get("started", ""))}' + (f' · namespace {html.escape(meta["ns"])}' if meta.get("ns") else "") \
          + f' · {t_end:.0f} s · {len(data["joints"])} joint samples'
    css = CSS.replace("%LIGHT%", "".join(f"--s{i}:{c};" for i, c in enumerate(SERIES_LIGHT))) \
             .replace("%DARK%", "".join(f"--s{i}:{c};" for i, c in enumerate(SERIES_DARK)))
    js = JS.replace("__DATA__", json.dumps(hv, separators=(",", ":")))
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(title or "armdoc report")}</title><style>{css}</style></head><body><main>
<h1>{html.escape(title or "armdoc report")}</h1><div class="sub">{sub}</div>
<div class="chips">{"".join(chips)}</div>
<div class="card" id="card">{svg}<div class="legend">{legend}</div><div id="tip"></div></div>
<h2>Findings</h2><div class="scroll"><table><tr><th>Level</th><th>Code</th><th>From (s)</th><th>To (s)</th><th>For (s)</th><th>Message</th></tr>{trs}</table></div>
<details><summary>Joint positions as a table (1 Hz)</summary><div class="scroll"><table>{joint_tbl}</table></div></details>
</main><script>{js}</script></body></html>"""


JS = """
(function(){var D=__DATA__,svg=document.getElementById('chart'),xh=document.getElementById('xh'),tip=document.getElementById('tip'),card=document.getElementById('card');
var off={};
document.querySelectorAll('.legend button').forEach(function(b){b.addEventListener('click',function(){var j=b.dataset.j;off[j]=!off[j];
b.classList.toggle('off',!!off[j]);b.setAttribute('aria-pressed',!off[j]);
document.querySelectorAll('.series[data-j="'+j+'"]').forEach(function(g){g.classList.toggle('off',!!off[j]);});});});
function near(t){var lo=0,hi=D.t.length-1;if(hi<0)return -1;while(lo<hi){var m=(lo+hi)>>1;if(D.t[m]<t)lo=m+1;else hi=m;}
if(lo>0&&Math.abs(D.t[lo-1]-t)<Math.abs(D.t[lo]-t))lo--;return lo;}
svg.addEventListener('mousemove',function(e){var r=svg.getBoundingClientRect(),vx=(e.clientX-r.left)/r.width*D.W;
if(vx<D.L||vx>D.W-D.R){xh.style.display='none';tip.style.display='none';return;}
var t=(vx-D.L)/(D.W-D.L-D.R)*D.t_end;xh.setAttribute('x1',vx);xh.setAttribute('x2',vx);xh.style.display='block';
var i=near(t),h='<b>t = '+t.toFixed(1)+' s</b>';
if(i>=0&&Math.abs(D.t[i]-t)<1.5){D.names.forEach(function(n,k){if(!off[k]&&D.p[i][k]!=null)h+='<br>'+n+': '+D.p[i][k].toFixed(3)+' rad';});}
else h+='<br>no joint data';
D.spans.forEach(function(s){if(t>=s.a&&t<=s.b)h+='<br><span class="'+(s.l=='ERROR'?'e':'w')+'">'+(s.l=='ERROR'?'●':'▲')+' '+s.c+'</span>';});
tip.innerHTML=h;tip.style.display='block';var cr=card.getBoundingClientRect(),x=e.clientX-cr.left+14;
if(x+tip.offsetWidth>cr.width)x=e.clientX-cr.left-tip.offsetWidth-14;tip.style.left=x+'px';tip.style.top=(e.clientY-cr.top+10)+'px';});
svg.addEventListener('mouseleave',function(){xh.style.display='none';tip.style.display='none';});})();
"""


# ---------------------------------------------------------------- PNG
def build_png(data, path, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    sp = spans(data["samples"], data["t_end"])
    cs = ctrl_spans(data["samples"], data["t_end"])
    ser = series(data)
    t_end = max(data["t_end"], 1.0)
    fig = plt.figure(figsize=(11, 5.6 + 0.0 * len(sp)), facecolor="#fcfcfb")
    gs = fig.add_gridspec(2 if cs else 1, 1, height_ratios=[10, 1] if cs else [1], hspace=0.12)
    ax = fig.add_subplot(gs[0])
    ax.set_facecolor("#f4f3f0")
    for i, n in enumerate(data["names"]):
        xs = [t for t, _ in ser[n]]
        ys = [float("nan") if v is None else v for _t, v in ser[n]]
        ax.plot(xs, ys, color=SERIES_LIGHT[i % 8], lw=2, label=n)
    ax.set_xlim(0, t_end)
    y0, y1 = ax.get_ylim()
    ax.set_ylim(y0 - (y1 - y0) * 0.14, y1)          # clear space for the legend under the lines
    ax.grid(color="#e4e3df", lw=1)
    for sp_ in ("top", "right"):
        ax.spines[sp_].set_visible(False)
    ax.set_ylabel("joint position (rad)", color="#52514e")
    if cs:
        ax.tick_params(labelbottom=False)
    else:
        ax.set_xlabel("time since start (s)", color="#52514e")
    rows, nrows = label_rows(sp, lambda t: t / t_end * 900, char_w=6.6)
    for s, r in zip(sp, rows):
        col = ERR_COLOR if s["level"] == "ERROR" else WARN_COLOR
        ax.axvspan(s["start"], max(s["end"], s["start"] + t_end * 0.002), color=col, alpha=0.16 if s["level"] == "ERROR" else 0.24, lw=0)
        ax.annotate(("● " if s["level"] == "ERROR" else "▲ ") + s["code"], xy=(s["start"], 1.0), xycoords=("data", "axes fraction"),
                    xytext=(3, 4 + 14 * r), textcoords="offset points", fontsize=8.5, fontweight="bold", color="#0b0b0b", va="bottom")
        ax.plot([s["start"]] * 2, [1.0, 1.0 + (4 + 14 * r) / 330], transform=ax.get_xaxis_transform(), color=col, lw=1.2, clip_on=False)
    ax.legend(loc="lower left", ncol=min(6, max(1, len(data["names"]))), frameon=False, fontsize=9, bbox_to_anchor=(0, -0.02))
    if cs:
        lane = fig.add_subplot(gs[1], sharex=ax)
        lane.set_ylim(0, 1)
        lane.set_yticks([])
        for sp_ in ("top", "right", "left"):
            lane.spines[sp_].set_visible(False)
        for c in cs:
            lane.axvspan(c["start"], max(c["end"], c["start"] + t_end * 0.002), ymin=0.1, ymax=0.9, color="#8a8984", alpha=0.3, lw=0)
            lane.text(c["start"] + t_end * 0.004, 0.5, f'{c["name"]}: {c["state"]}', fontsize=8, va="center", color="#52514e")
        lane.set_xlabel("time since start (s)", color="#52514e")
    top = 0.84 - 0.03 * (nrows - 1)
    fig.subplots_adjust(left=0.07, right=0.985, top=top, bottom=0.12)
    fig.text(0.07, 0.965, title or "armdoc report", fontsize=14, fontweight="bold", color="#0b0b0b", va="top")
    fig.savefig(path, dpi=140, facecolor=fig.get_facecolor())
    plt.close(fig)


# ---------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(prog="armdoc report", description="Render an `armdoc --record` file as a timeline.")
    ap.add_argument("jsonl")
    ap.add_argument("--html", metavar="FILE", help="write one self-contained HTML report")
    ap.add_argument("--png", metavar="FILE", help="write a PNG (needs matplotlib)")
    ap.add_argument("--title")
    a = ap.parse_args(argv)
    if not a.html and not a.png:
        a.html = a.jsonl.rsplit(".", 1)[0] + ".html"
    data = load(a.jsonl)
    if not data["samples"] and not data["joints"]:
        print(f"armdoc report: no records in {a.jsonl}", file=sys.stderr)
        return 2
    if a.html:
        with open(a.html, "w") as fh:
            fh.write(build_html(data, a.title))
        print(f"wrote {a.html}")
    if a.png:
        build_png(data, a.png, a.title)
        print(f"wrote {a.png}")
    sm = summary(data)
    print("errors:", ", ".join(sm["errors"]) or "none", "| warnings:", ", ".join(sm["warnings"]) or "none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
