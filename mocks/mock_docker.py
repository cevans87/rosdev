from unittest.mock import MagicMock


class MockDocker(MagicMock):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
