from __future__ import annotations

from .base import Api


class UiApi(Api):
    def display_message(self, msg: str) -> None:
        """
        Displays a toast message in the user interface of the simulator.

        Args:
            msg: The message to display.
        """
        data = dict(type='DisplayGuiMessage')
        data['message'] = msg
        self._send(data).ack('GuiMessageDisplayed')

    def hide_hud(self) -> None:
        """
        Hides the HUD in the simulator.
        """
        data = dict(type='HideHUD')
        self._send(data)

    def show_hud(self) -> None:
        """
        Shows the HUD in the simulator.
        """
        data = dict(type='ShowHUD')
        self._send(data)
