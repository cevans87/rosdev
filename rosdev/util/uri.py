from dataclasses import dataclass, field
from urllib.parse import ParseResult, urlparse

from rosdev.util.path import Path


@dataclass(frozen=True)
class Uri:
    _parse_result: ParseResult = field(init=False)
    
    def __init__(self, _uri: str) -> None:
        object.__setattr__(self, '_parse_result', urlparse(_uri))

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}('{self}')"

    def __str__(self) -> str:
        return self._parse_result.geturl()

    @property
    def hostname(self) -> str:
        return self._parse_result.hostname

    @property
    def password(self) -> str:
        return self._parse_result.password

    @property
    def path(self) -> Path:
        return Path(self._parse_result.path)

    @property
    def port(self) -> int:
        return self._parse_result.port

    @property
    def scheme(self) -> str:
        return self._parse_result.scheme

    @property
    def username(self) -> str:
        return self._parse_result.username
