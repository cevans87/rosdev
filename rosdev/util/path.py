from __future__ import annotations
from logging import getLogger as _getLogger
import os
from pathlib import PosixPath
from typing import Optional


log = _getLogger(__name__)


class Path(PosixPath):

    _rosdev: Optional[Path] = None
    _store: Optional[Path] = None
    _workspace: Optional[Path] = None

    def __bool__(self) -> bool:
        return self != Path() and bool(super())

    def read_bytes(self) -> bytes:
        data = super().read_bytes()

        log.debug(f'Read from {self}, data: \n{data.decode()}')
        
        return data

    def read_text(self, encoding=None, errors=None):
        data = super().read_text(encoding=encoding, errors=errors)

        log.debug(f'Read from {self}, data: \n{data}')

        return data

    @classmethod
    def rosdev(cls) -> Path:
        if cls._rosdev is None:
            global_path = cls.home() / '.rosdev' / cls.workspace().relative_to(cls.home())
            if not global_path.is_dir():
                global_path.mkdir(parents=True)

            path = cls.workspace() / '.rosdev'

            if path.exists() or path.is_symlink():
                path.unlink()
            path.symlink_to(os.path.relpath(f'{global_path}', f'{cls.cwd()}'))

            cls._rosdev = path

        return cls._rosdev

    @classmethod
    def store(cls) -> Path:
        assert cls._store is not None

        return cls._store

    @classmethod
    def set_store(cls, path: Path) -> None:
        if path.is_absolute():
            assert path.relative_to(cls.rosdev())
        else:
            path = cls.rosdev() / path

        if not path.is_dir():
            path.mkdir(parents=True)
            
        cls._store = path

    @classmethod
    def volume(cls) -> Path:
        return cls.store() / 'volume'

    @classmethod
    def workspace(cls) -> Path:
        if cls._workspace is None:
            # FIXME find workspace based on ~/.rosdev/a/b
            for path in [cls.cwd(), *cls.cwd().parents]:
                if path == cls.home():
                    break
                if (path / '.rosdev').is_dir():
                    break

            if path == Path.home():
                path = cls.cwd()

            assert cls.home() in path.parents, 'Workspace must be in a subdirectory of home.'

            cls._workspace = path

        return cls._workspace

    def write_bytes(self, data: bytes) -> None:
        log.debug(f'Writing to {self}, bytes:\n{data.decode()}')
        
        self.parent.mkdir(parents=True, exist_ok=True)
        
        super().write_bytes(data)

    def write_text(self, data, encoding=None, errors=None):
        log.debug(f'Writing to {self}, text: \n{data}')

        self.parent.mkdir(parents=True, exist_ok=True)
        super().write_text(data, encoding=encoding, errors=errors)
