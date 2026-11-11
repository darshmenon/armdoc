import json
import math

from armdoc.record import Recorder
from armdoc.report import build_html, build_png, ctrl_spans, label_rows, load, main, nice_ticks, series, spans, summary


def synth(path):
    """A 20 s run: healthy, controller inactive 5-9 s, joint_states silent 12-17 s."""
    with open(path, "w") as f:
        f.write(json.dumps({"type": "meta", "started": "2026-01-01T00:00:00", "ns": "", "limits": {}}) + "\n")
        for i in range(0, 200):
            t = i / 10
            if 12 <= t < 17:
                continue
            pos = {"j1": math.sin(t), "j2": math.cos(t)}
            f.write(json.dumps({"type": "joints", "t": t, "pos": pos}) + "\n")
        for t in range(0, 21):
            fs, ctrl = [], {"arm": "active"}
            if 5 <= t < 9:
                fs += [{"level": "ERROR", "code": "CTRL_INACTIVE", "msg": "arm inactive"}]
                ctrl = {"arm": "inactive"}
            if 14 <= t < 18:
                fs += [{"level": "ERROR", "code": "JOINT_STATES_STALE", "msg": "silent"}]
            if t == 3:
                fs += [{"level": "WARN", "code": "NEAR_LIMIT", "msg": "j1 near limit"}]
            f.write(json.dumps({"type": "findings", "t": t, "findings": fs, "ctrl": ctrl}) + "\n")


def test_spans_and_ctrl_spans(tmp_path):
    p = tmp_path / "r.jsonl"
    synth(p)
    d = load(p)
    sp = {s["code"]: s for s in spans(d["samples"], d["t_end"])}
    assert (sp["CTRL_INACTIVE"]["start"], sp["CTRL_INACTIVE"]["end"]) == (5, 9)
    assert (sp["JOINT_STATES_STALE"]["start"], sp["JOINT_STATES_STALE"]["end"]) == (14, 18)
    assert sp["NEAR_LIMIT"]["level"] == "WARN" and sp["NEAR_LIMIT"]["end"] == 4
    c = ctrl_spans(d["samples"], d["t_end"])
    assert [(x["name"], x["start"], x["end"]) for x in c] == [("arm", 5, 9)]
    sm = summary(d)
    assert sm["errors"] == ["CTRL_INACTIVE", "JOINT_STATES_STALE"] and sm["warnings"] == ["NEAR_LIMIT"]


def test_open_span_runs_to_end():
    samples = [(0, [], {}), (1, [{"level": "ERROR", "code": "X", "msg": "m"}], {}), (2, [{"level": "ERROR", "code": "X", "msg": "m"}], {})]
    assert spans(samples, 2.5)[0]["end"] == 2.5


def test_lines_break_at_gaps(tmp_path):
    p = tmp_path / "r.jsonl"
    synth(p)
    pts = series(load(p))["j1"]
    assert any(v is None for _t, v in pts)             # the 12-17 s silence is a gap, not a straight line


def test_label_rows_avoid_overlap():
    sp = [{"start": 0.0, "code": "AAAAAAAAAAAA"}, {"start": 0.1, "code": "BBBBBBBB"}, {"start": 50.0, "code": "CC"}]
    rows, n = label_rows(sp, lambda t: t * 10)
    assert rows[0] != rows[1] and rows[2] == 0 and n == 2


def test_nice_ticks_cover_range():
    t = nice_ticks(-0.3, 2.9)
    assert t[0] <= -0.3 and t[-1] >= 2.9 and len(t) <= 12
    assert len(nice_ticks(1.0, 1.0)) >= 2
    assert nice_ticks(-2.8, 2.5)[-1] >= 2.5          # regression: ticks stopped at 2 and the data poked out


def test_html_is_self_contained_and_labelled(tmp_path):
    p = tmp_path / "r.jsonl"
    synth(p)
    h = build_html(load(p), "demo")
    assert "<svg" in h and "CTRL_INACTIVE" in h and "JOINT_STATES_STALE" in h and "NEAR_LIMIT" in h
    assert "http://" not in h.replace("http://www.w3.org", "") and "https://" not in h
    assert "<script src" not in h and "<link" not in h
    assert h.count('data-j="0"') >= 2                      # legend button + series group
    assert "controller" in h                               # the inactive-controller lane


def test_healthy_run_says_so(tmp_path):
    p = tmp_path / "r.jsonl"
    p.write_text(json.dumps({"type": "joints", "t": 0.0, "pos": {"j": 0.1}}) + "\n"
                 + json.dumps({"type": "findings", "t": 1.0, "findings": [], "ctrl": {}}) + "\n")
    assert "no findings" in build_html(load(p))


def test_cli_writes_html_and_png(tmp_path, capsys):
    p = tmp_path / "r.jsonl"
    synth(p)
    assert main([str(p), "--html", str(tmp_path / "o.html"), "--png", str(tmp_path / "o.png")]) == 0
    assert (tmp_path / "o.html").stat().st_size > 2000
    assert (tmp_path / "o.png").read_bytes()[:4] == b"\x89PNG"
    assert "CTRL_INACTIVE" in capsys.readouterr().out
    empty = tmp_path / "e.jsonl"
    empty.write_text("")
    assert main([str(empty)]) == 2


def test_recorder_throttles_and_roundtrips(tmp_path):
    p = tmp_path / "rec.jsonl"
    r = Recorder(str(p), ns="/ur3", joint_hz=10)
    for i in range(100):
        r.joints(i * 0.01, {"j1": 0.5, "bad": float("nan")})   # 1 s at 100 Hz
    from armdoc.core import Finding
    r.findings(1.0, [Finding("ERROR", "X", "boom")], [("arm", "t/Type", "inactive")])
    r.set_limits({"j1": {"lower": -1}})
    r.close()
    d = load(p)
    assert 9 <= len(d["joints"]) <= 11 and d["names"] == ["j1"]       # NaN dropped, ~10 Hz
    assert d["meta"]["ns"] == "/ur3" and d["limits"]["j1"]["lower"] == -1
    assert d["samples"][0][1][0]["code"] == "X" and d["samples"][0][2] == {"arm": "inactive"}


def test_never_active_controller_is_not_a_fault():
    samples = [(0, [], {"arm": "active", "spare": "inactive"}), (1, [], {"arm": "inactive", "spare": "inactive"}),
               (2, [], {"arm": "active", "spare": "inactive"})]
    assert [(c["name"], c["start"], c["end"]) for c in ctrl_spans(samples, 3)] == [("arm", 1, 2)]
