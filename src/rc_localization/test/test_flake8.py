from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
@pytest.mark.filterwarnings('ignore:SelectableGroups')  # flake8 4 on Python 3.10
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '\n'.join(errors)
