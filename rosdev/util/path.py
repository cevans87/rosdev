from __future__ import annotations
from logging import getLogger as _getLogger
from pathlib import PosixPath
from typing import Optional


log = _getLogger(__name__)


class Path(PosixPath):

    _workspace_path: Optional[Path] = None

    def __bool__(self) -> bool:
        return self != Path() and bool(super())

    @classmethod
    def db(cls) -> Path:
        return cls.workspace() / '.rosdev' / 'db'
    
    def read_bytes(self) -> bytes:
        data = super().read_bytes()

        log.debug(f'Read from {self}, data: \n{data.decode()}')
        
        return data

    def read_text(self, encoding=None, errors=None):
        data = super().read_text(encoding=encoding, errors=errors)

        log.debug(f'Read from {self}, data: \n{data}')

        return data

    @classmethod
    def workspace(cls) -> Path:
        if cls._workspace_path is None:
            for path in [cls.cwd(), *cls.cwd().parents]:
                if path == cls.home():
                    break
                if (path / '.rosdev').is_dir():
                    break
            else:
                path = cls.cwd()

            assert cls.home() in path.parents, 'Workspace must be in a subdirectory of home.'

            cls._workspace_path = path

        return cls._workspace_path

    def write_bytes(self, data: bytes) -> None:
        log.debug(f'Writing to {self}, bytes:\n{data.decode()}')
        
        self.parent.mkdir(parents=True, exist_ok=True)
        
        super().write_bytes(data)

    def write_text(self, data, encoding=None, errors=None):
        log.debug(f'Writing to {self}, text: \n{data}')

        self.parent.mkdir(parents=True, exist_ok=True)
        super().write_text(data, encoding=encoding, errors=errors)
