import py
import pytest
@pytest.fixture(scope="session")
def test_data_dir():
    this_dir = py.path.local(__file__).dirpath()
    tdd = this_dir.join('test_data').ensure(dir=True)
    return tdd
