from __future__ import annotations

from .base import Api


# The World Editor keeps two kinds of unsaved state: the level itself
# (`editor.dirty`) and per-tool changes (`editor.dirtyTools`, one save function
# per dirty tool, so an empty table means no dirty tool). The engine's own
# "don't save" answer to its save prompt clears both, and that is what these
# chunks do. They report the state as it was before any of it was dropped.
_IS_DIRTY_CHUNK = """
if not editor then return false end
if editor.dirty then return true end
return editor.dirtyTools ~= nil and next(editor.dirtyTools) ~= nil
"""

_DISCARD_CHANGES_CHUNK = """
if not editor then return false end
local dirty = (editor.dirty == true)
if not dirty and editor.dirtyTools ~= nil then dirty = next(editor.dirtyTools) ~= nil end
if editor.resetDirty then editor.resetDirty() end
if editor.resetAllDirtyTools then editor.resetAllDirtyTools() end
return dirty
"""


def _to_bool(value: object) -> bool:
    """Convert the simulator's stringified chunk result to a bool."""
    return str(value).strip().lower() == "true"


class EditorApi(Api):
    """
    An API for the state of the simulator's World Editor.

    The World Editor tracks unsaved changes to the loaded level. While it has
    them, every level load and every simulator shutdown runs its save prompt
    first, which waits on an answer a headless simulator cannot give it. This
    API reports and drops that state.

    Args:
        beamng: An instance of the simulator.
    """

    def is_dirty(self) -> bool:
        """
        Checks whether the World Editor has unsaved level or tool changes.

        Returns:
            ``True`` if something is waiting to be saved, ``False`` otherwise.
            ``False`` is also returned when no World Editor state exists, which
            is the case before a level has been loaded.
        """
        resp = self._beamng.control.queue_lua_command(_IS_DIRTY_CHUNK, response=True)
        return _to_bool(resp)

    def discard_changes(self) -> bool:
        """
        Drops the unsaved World Editor changes of the loaded level without
        writing anything to disk. Level state and tool state are both cleared,
        exactly like declining the editor's save prompt.

        Anything the simulator has not saved is lost: road imports, terrain
        edits and other level changes made through the editor are only present
        in the running simulator, not in the level folder.

        Returns:
            ``True`` if there were changes to drop, ``False`` if there was
            nothing to drop.
        """
        resp = self._beamng.control.queue_lua_command(
            _DISCARD_CHANGES_CHUNK, response=True
        )
        return _to_bool(resp)
