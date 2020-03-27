from atools import memoize

from rosdev.util.path import Path

memoize_db = memoize(db_path=Path.db())
