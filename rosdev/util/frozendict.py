from frozendict import frozendict as _frozendict
from typing import Mapping


FrozenDict = Mapping


# noinspection PyPep8Naming
class frozendict(_frozendict):
    
    def __str__(self) -> str:
        return f'{self.__class__.__name__}({self._dict})'
