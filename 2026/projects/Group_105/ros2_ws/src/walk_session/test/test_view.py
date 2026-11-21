"""Checks of the session panel drawn in RViz."""

from walk_session import view
from walk_session.session import Progress, Walk

CUE = 'Take your time, start again when ready'


def walk(**changes):
    numbers = {'number': 2, 'measured': True, 'duration_s': 5.14, 'moving_s': 5.0, 'steps': 8,
               'distance_m': 3.55, 'stops': 0}
    numbers.update(changes)
    return Walk(**numbers)


def test_before_the_first_walk():
    assert view.panel_text('20261121-163000', None, None, CUE) \
        == 'Session 20261121-163000\nWaiting for a walk to start'


def test_while_walking():
    text = view.panel_text('s', Progress(3, 12, 0, False), walk(), CUE)
    assert text.splitlines() == [
        'Session s',
        'Walk 3: walking, 12 steps, 0 stops',
        'Last walk (2): 5.1 s, 8 steps, 96 steps/min, 0.71 m/s, 0 stops',
    ]


def test_while_stopped_the_cue_is_shown():
    lines = view.panel_text('s', Progress(3, 1, 1, True), None, CUE).splitlines()
    assert lines[1:] == ['Walk 3: stopped, 1 step, 1 stop', f'Said: "{CUE}"']


def test_a_walk_not_measured():
    assert view.walk_numbers(walk(measured=False)) == 'not measured, 5.1 s'


def test_unknown_numbers():
    assert view.walk_numbers(walk(moving_s=0.0, steps=0)) \
        == '5.1 s, 0 steps, ? steps/min, ? m/s, 0 stops'
