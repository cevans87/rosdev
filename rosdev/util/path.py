from __future__ import annotations
from logging import getLogger as _getLogger
import os
from pathlib import PosixPath
from typing import Optional


log = _getLogger(__name__)


class Path(PosixPath):
    _db: Optional[Path] = None
    _docker: Optional[Path] = None
    _rosdev: Optional[Path] = None
    _store: Optional[Path] = None
    _volume: Optional[Path] = None
    _workspace: Optional[Path] = None

    def __bool__(self) -> bool:
        return self != Path() and bool(super())

    @classmethod
    def db(cls) -> Path:
        if cls._db is None:
            cls._db = cls.store() / 'db'

        return cls._db
    
    @classmethod
    def docker(cls) -> Path:
        if cls._docker is None:
            cls._docker = cls.volume() / 'docker'

        return cls._docker

    @classmethod
    def rosdev(cls) -> Path:
        if cls._rosdev is None:
            global_path = Path.home() / '.rosdev' / cls.workspace().relative_to(Path.home())
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
        if cls._store is None:
            from rosdev.util.options import Options
            options = Options.of_args()
            cls._store = cls.rosdev() / options.release / options.architecture
            if not cls._store.exists():
                cls._store.mkdir(parents=True)
                
        return cls._store

    @classmethod
    def volume(cls) -> Path:
        if cls._volume is None:
            cls._volume = cls.store() / 'volume'

        return cls._volume

    @classmethod
    def workspace(cls) -> Path:
        if cls._workspace is None:
            for path in [Path.cwd(), *Path.cwd().parents]:
                if path == Path.home():
                    break
                if (path / '.rosdev').is_dir():
                    break

            if path == Path.home():
                path = Path.cwd()

            assert Path.home() in path.parents, 'Workspace must be in a subdirectory of home.'

            cls._workspace = path
        
        return cls._workspace

    def read_bytes(self) -> bytes:
        data = super().read_bytes()

        log.debug(f'Read from {self}, data: \n{data.decode()}')

        return data

    def read_text(self, encoding=None, errors=None):
        data = super().read_text(encoding=encoding, errors=errors)

        log.debug(f'Read from {self}, data: \n{data}')

        return data

    def write_bytes(self, data: bytes) -> None:
        log.debug(f'Writing to {self}, bytes:\n{data.decode()}')

        self.parent.mkdir(parents=True, exist_ok=True)

        super().write_bytes(data)

    def write_text(self, data, encoding=None, errors=None):
        log.debug(f'Writing to {self}, text: \n{data}')

        self.parent.mkdir(parents=True, exist_ok=True)

        super().write_text(data, encoding=encoding, errors=errors)
