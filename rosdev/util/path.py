from logging import getLogger as _getLogger
from pathlib import PosixPath


log = _getLogger(__name__)


class Path(PosixPath):

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

    def write_bytes(self, data: bytes) -> None:
        log.debug(f'Writing to {self}, bytes:\n{data.decode()}')
        
        self.parent.mkdir(parents=True, exist_ok=True)
        
        super().write_bytes(data)

    def write_text(self, data, encoding=None, errors=None):
        log.debug(f'Writing to {self}, text: \n{data}')

        self.parent.mkdir(parents=True, exist_ok=True)
        super().write_text(data, encoding=encoding, errors=errors)
