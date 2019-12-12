import pytest
from typing import Optional

from rosdev.util.path import Path
from rosdev.util.uri import Uri


@pytest.fixture(params=['localhost', '172.0.0.1'])
def hostname(request) -> str:
    yield request.param


@pytest.fixture(params=['', 'foo_password'])
def password(request) -> str:
    yield request.param


@pytest.fixture(params=[Path('/foo/bar/baz'), None])
def path(request) -> Optional[Path]:
    yield request.param


@pytest.fixture(params=[22])
def port(request) -> int:
    yield request.param


def test_uri_no_scheme_leaves_out_scheme() -> None:
    assert Uri('foo@bar:/foo/bar/baz').scheme == ''


def test_uri_parses_hostname(hostname: str) -> None:
    assert Uri(f'ssh://{hostname}:22').hostname == hostname


def test_uri_parses_password(password: str) -> None:
    assert Uri(f'ssh://user{":" + password if password else ""}@host:22').password == password


def test_uri_parses_sftp_path(path: Optional[Path]) -> None:
    assert Uri(f'sftp://user@host{":" + f"{path}" if path is not None else ""}').path == path


def test_uri_parses_port(port: Optional[int]) -> None:
    assert Uri(f'ssh://foo@bar{":" + f"{port}" if port is not None else ""}').port == port


def test_uri_hash_is_consistent() -> None:
    assert hash(Uri('ssh://a@b')) == hash(Uri('ssh://a@b'))
