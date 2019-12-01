import pytest
from rosdev.util.path import Path
from rosdev.util.uri import Uri


@pytest.fixture(params=['localhost', '172.0.0.1'])
def hostname(request) -> str:
    yield request.param


@pytest.fixture(params=[Path('/foo/bar/baz')])
def path(request) -> Path:
    yield request.param


@pytest.fixture(params=[22])
def port(request) -> int:
    yield request.param


def test_uri_no_scheme_leaves_out_scheme() -> None:
    assert Uri('foo@bar:/foo/bar/baz').scheme == ''


def test_uri_parses_hostname(hostname: str) -> None:
    assert Uri(f'ssh://{hostname}:22').hostname == hostname


def test_uri_parses_sftp_path(path: Path) -> None:
    assert Uri(f'sftp://foo@bar:{path}').path == path


def test_uri_parses_port(port: int) -> None:
    assert Uri(f'ssh://foo@bar:{port}').port == port
