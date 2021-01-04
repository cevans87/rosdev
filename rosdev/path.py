from __future__ import annotations
from logging import getLogger
from pathlib import PosixPath
from textwrap import indent
from typing import Union


log = getLogger(__name__)


class Path(PosixPath):
    _workspace: Path = None

    @classmethod
    def cache(cls) -> Path:
        return cls.home() / '.cache' / 'rosdev'
    
    @classmethod
    def config(cls) -> Path:
        return cls.home() / '.config' / 'rosdev'

    @classmethod
    def build(cls, path: Union[Path, str] = '/') -> Path:
        return Path('/build') / path
    
    def context(self) -> Path:
        if self.is_absolute():
            return self.relative_to(self.workspace())
        else:
            return self
    
    def image_build(self) -> Path:
        return Path.build(self.context())

    def image_source(self) -> Path:
        return Path.source(self.context())

    @classmethod
    def memoize_db(cls) -> Path:
        return cls.config() / 'memoize_db'

    @classmethod
    def source(cls, path: Union[Path, str] = '/') -> Path:
        return Path('/source') / path

    @classmethod
    def workspace(cls) -> Path:
        if cls._workspace is None:
            workspace = Path.cwd()
            while (
                    (Path.home() in workspace.parents)
                    and not ((workspace / '.rosdev').exists())
            ):
                workspace = workspace.parent
                
                log.debug(f'{workspace = }')
                
                cls._workspace = workspace
                
        return cls._workspace

    def write_bytes(self, data: bytes) -> None:
        log.debug(f'Write to {self}, bytes:\n{indent(data.decode(), "    ")}')
        self.parent.mkdir(parents=True, exist_ok=True)
        super().write_bytes(data)

    def write_text(self, data: str, encoding=None, errors=None) -> None:
        log.debug(f'Writing to {self}, text: \n{indent(data, "    ")}')
        self.parent.mkdir(parents=True, exist_ok=True)
        super().write_text(data, encoding=encoding, errors=errors)