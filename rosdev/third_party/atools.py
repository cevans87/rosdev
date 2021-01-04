from atools import memoize

from rosdev.path import Path as _Path


_Path.memoize_db().parent.mkdir(parents=True, exist_ok=True)
memoize_db = memoize(db_path=_Path.memoize_db())
