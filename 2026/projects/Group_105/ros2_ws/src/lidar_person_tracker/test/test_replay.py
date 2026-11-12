"""Checks of the settings given to the replay tool."""

from lidar_person_tracker.replay import parse_settings
import pytest


def test_settings_get_the_right_types():
    settings = parse_settings(['path_distance_m=1.2', 'background_scans=18'])
    assert settings == {'path_distance_m': 1.2, 'background_scans': 18}
    assert isinstance(settings['background_scans'], int)


@pytest.mark.parametrize('pair', ['speed=1', 'path_distance_m', 'background_scans=1.5'])
def test_bad_settings_are_rejected(pair):
    with pytest.raises(ValueError):
        parse_settings([pair])
