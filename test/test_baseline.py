

def test_parse_expect_and_check():
    import pytest
    from armdoc import baseline as bl
    assert bl.parse_expect(['a:10', '/b=2.5']) == {'a': {'hz': 10.0, 'jitter': 0.0, 'n': 0},
                                                  'b': {'hz': 2.5, 'jitter': 0.0, 'n': 0}}
    for bad in ('a', 'a:x', 'a:0', ':5'):
        with pytest.raises(ValueError):
            bl.parse_expect([bad])
    t = [i / 10 for i in range(200)]
    assert bl.check(bl.parse_expect(['a:10']), {'a': t}, 19.9) == []
    assert bl.check(bl.parse_expect(['a:50']), {'a': t}, 19.9)[0][1] == 'RATE_DROP'


def test_load_rejects_bad_files_with_a_readable_error(tmp_path):
    import pytest
    from armdoc import baseline as bl
    cases = {'missing.json': None, 'empty.json': '', 'notjson.json': 'hello', 'nokey.json': '{"version": 1}',
             'nohz.json': '{"streams": {"a": {"jitter": 1}}}', 'zero.json': '{"streams": {"a": {"hz": 0}}}',
             'list.json': '[1, 2]'}
    for name, txt in cases.items():
        p = tmp_path / name
        if txt is not None:
            p.write_text(txt)
        with pytest.raises(ValueError):
            bl.load(str(p))
    good = tmp_path / 'good.json'
    bl.save({'a': {'hz': 10.0, 'jitter': 0.0, 'n': 5}}, str(good))
    assert bl.load(str(good))['a']['hz'] == 10.0
