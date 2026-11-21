"""Checks of the session panel shown in RViz."""

from walk_session import view
from walk_session.session import Progress


def test_before_the_first_walk():
    assert view.panel_lines(None, 0) == ['No walk yet']


def test_between_walks():
    assert view.panel_lines(None, 1) == ['1 walk done']
    assert view.panel_lines(None, 3) == ['3 walks done']


def test_while_walking():
    assert view.panel_lines(Progress(3, 12, 0, False), 3) \
        == ['Walk 3: walking', '12 steps, 0 stops']


def test_while_stopped():
    assert view.panel_lines(Progress(3, 1, 1, True), 3) == ['Walk 3: stopped', '1 step, 1 stop']


def test_the_panel_is_read_as_html():
    assert view.panel_text(['a < b & c', 'x']) == 'a &lt; b &amp; c\nx'
    assert view.panel_height_px(['a', 'b']) > view.panel_height_px(['a'])
