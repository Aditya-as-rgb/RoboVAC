# Copyright 2026 Aditya Choure
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run a blocking call off the GUI thread and deliver its result back on it.

Stopping a stack waits on SIGINT / SIGTERM timeouts of several seconds per process. Done
on the GUI thread that freezes the window ("Not Responding") for as long as it takes, and
no button can show a busy state while the event loop is stuck. ``Background.run`` moves
the call to a daemon thread; the ``then`` callback is invoked on the GUI thread through a
queued signal, so it may touch widgets.
"""

import threading
import traceback

from PySide6.QtCore import QObject, Signal


class Background(QObject):
    _done = Signal(object, object, object)       # then, result, error

    def __init__(self, parent=None):
        super().__init__(parent)
        self._done.connect(self._deliver)

    def run(self, fn, then=None, name='background'):
        """Call ``fn()`` on a worker thread, then ``then(result, error)`` on the GUI thread.

        ``error`` is the exception ``fn`` raised (``result`` is then None), else None.
        """
        def work():
            try:
                result, error = fn(), None
            except Exception as exc:      # noqa: BLE001 - handed to the callback
                result, error = None, exc
            self._done.emit(then, result, error)

        thread = threading.Thread(target=work, name=name, daemon=True)
        thread.start()
        return thread

    def post(self, fn):
        """Run ``fn()`` on the GUI thread; callable from any thread (e.g. a process-exit
        callback), which is the only safe way for it to touch a widget."""
        self._done.emit(lambda _result, _error: fn(), None, None)

    def _deliver(self, then, result, error):
        if then is None:
            return
        try:
            then(result, error)
        except Exception:      # noqa: BLE001 - a bad callback must not kill the event loop
            traceback.print_exc()
