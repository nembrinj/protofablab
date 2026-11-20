"""
Turn session results into the JSON messages the dashboard reads.

This module has no ROS imports, so it can be tested without ROS.
"""

import json
import math

TOPIC_PREFIX = 'rehab'


def walk_topic(patient_id):
    """Return the MQTT topic of the walks; raise ValueError if the ID cannot be in a topic."""
    if not patient_id or any(char in patient_id for char in '/+#') \
            or not patient_id.isprintable():
        raise ValueError(f'patient_id {patient_id!r} cannot be part of an MQTT topic')
    return f'{TOPIC_PREFIX}/{patient_id}/walk'


def walk_payload(summary):
    """Return the JSON text for one WalkSummary; numbers that are not known are left out."""
    values = {
        'session_id': summary.session_id,
        'walk': summary.walk,
        'measured': summary.measured,
        'duration_s': round(summary.duration_s, 2),
        'steps': summary.steps,
        'cadence_spm': round(summary.cadence_spm, 1),
        'speed_mps': round(summary.speed_mps, 2),
        'stops': summary.stops,
    }
    # JSON has no NaN, and the dashboard shows a missing number as a gap.
    return json.dumps({name: value for name, value in values.items()
                       if not (isinstance(value, float) and not math.isfinite(value))})
