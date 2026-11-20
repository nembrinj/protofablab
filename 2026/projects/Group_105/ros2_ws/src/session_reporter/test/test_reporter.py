"""Checks of the messages sent to the dashboard."""

import json
import math
from types import SimpleNamespace

import pytest
from session_reporter.reporter import walk_payload, walk_topic


def summary(**changes):
    values = {'session_id': '20261120-140301', 'walk': 3, 'measured': True,
              'duration_s': 4.4167, 'steps': 7, 'cadence_spm': 95.1234, 'speed_mps': 0.6789,
              'stops': 1}
    values.update(changes)
    return SimpleNamespace(**values)


def test_walk_payload_has_the_dashboard_fields():
    assert json.loads(walk_payload(summary())) == {
        'session_id': '20261120-140301', 'walk': 3, 'measured': True, 'duration_s': 4.42,
        'steps': 7, 'cadence_spm': 95.1, 'speed_mps': 0.68, 'stops': 1,
    }


def test_unknown_numbers_are_left_out():
    payload = json.loads(walk_payload(summary(cadence_spm=math.nan, speed_mps=math.inf)))
    assert 'cadence_spm' not in payload
    assert 'speed_mps' not in payload
    assert payload['steps'] == 7


def test_walk_topic():
    assert walk_topic('p01') == 'rehab/p01/walk'


@pytest.mark.parametrize('patient_id', ['', 'p/01', 'p+', '#', 'p\n01'])
def test_bad_patient_ids_are_rejected(patient_id):
    with pytest.raises(ValueError):
        walk_topic(patient_id)
